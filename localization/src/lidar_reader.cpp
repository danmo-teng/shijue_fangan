#include "lidar_reader.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// Python's % has the divisor's positive sign (including a positive zero).
double mod(double value, double divisor) {
    double result = std::fmod(value, divisor);
    if (result < 0) result += divisor;
    return result == 0 ? 0 : result;
}

double round_even(double value) {
    const double lower = std::floor(value), fraction = value - lower;
    if (fraction < .5) return lower;
    if (fraction > .5) return lower + 1;
    return std::fmod(lower, 2) == 0 ? lower : lower + 1;
}

const std::array<uint8_t, 256> crc_table = [] {
    std::array<uint8_t, 256> table{};
    for (unsigned i = 0; i < 256; ++i) {
        unsigned value = i;
        for (unsigned j = 0; j < 8; ++j)
            value = (value & 0x80) ? ((value << 1) ^ 0x4d) & 255 : (value << 1) & 255;
        table[i] = static_cast<uint8_t>(value);
    }
    return table;
}();

uint8_t crc(const uint8_t *data, size_t n) {
    uint8_t result = 0;
    for (size_t i = 0; i < n; ++i) result = crc_table[result ^ data[i]];
    return result;
}

unsigned le16(const uint8_t *data) { return unsigned(data[0]) | (unsigned(data[1]) << 8); }

struct Event {
    enum Kind { Scan, Status, Ack, Crc } kind;
    uint64_t offset = 0;
    double first = 0, last = 0;
    unsigned tick = 0, count = 0;
    std::array<uint16_t, 31> distances{};
    std::array<uint8_t, 31> intensities{};
};

struct TimingError : std::runtime_error {
    bool epoch;
    TimingError(const char *message, bool new_epoch = false)
        : std::runtime_error(message), epoch(new_epoch) {}
};

struct TimestampValueError {};

class Reader {
public:
    explicit Reader(const lidar_reader_config_v1 &config) : c(config) {}
    lidar_reader_status_v1 status{};

    void fault(const char *message) {
        status.needs_new_epoch = 1;
        error(message);
        buffer.clear();
        reset_sector();
    }

    int feed(const uint8_t *data, size_t size, int64_t arrival,
             lidar_reader_scan_v1 callback, void *user) {
        if (status.needs_new_epoch) {
            return LIDAR_READER_SOURCE_EPOCH;
        }
        if (last_arrival >= 0 && arrival < last_arrival) {
            fault("receive timestamp reversal"); return LIDAR_READER_SOURCE_EPOCH;
        }
        if (size > c.max_rx_chunk_bytes || size > c.max_buffer_bytes - buffer.size()) {
            fault("bounded reader input/buffer budget exceeded"); return LIDAR_READER_SOURCE_EPOCH;
        }
        last_arrival = arrival;
        ++status.counters[0]; status.counters[1] += size;
        if (size) buffer.insert(buffer.end(), data, data + size);
        byte_count += size;
        auto records = parse();
        if (!clock_ready) {
            try { prime(records, arrival); }
            catch (const TimingError &e) {
                fault(e.what()); return LIDAR_READER_SOURCE_EPOCH;
            }
        }
        size_t emitted = 0;
        for (const auto &event : records) {
            if (event.kind == Event::Crc) {
                ++status.counters[5]; reset_segment(); continue;
            }
            if (event.kind != Event::Scan) {
                ++status.counters[event.kind == Event::Status ? 3 : 4]; continue;
            }
            ++status.counters[2];
            std::array<lidar_reader_point_v1, 31> points{};
            size_t count = 0;
            try { count = timed_points(event, arrival, points); }
            catch (const TimingError &e) {
                ++status.counters[6]; error(e.what());
                if (e.epoch) { fault(e.what()); return LIDAR_READER_SOURCE_EPOCH; }
                reset_segment(); continue;
            }
            std::vector<std::vector<lidar_reader_point_v1>> ready;
            try { sector(points, count, ready); }
            catch (const TimingError &e) {
                ++status.counters[7]; error(e.what()); reset_segment(); continue;
            }
            // The reference returns sectors only after the entire packet's
            // points pass sector processing. A later packet may still fault.
            for (const auto &scan : ready) {
                if (!callback(user, scan.data(), scan.size())) return LIDAR_READER_CALLBACK;
                ++emitted;
            }
        }
        status.counters[8] += emitted;
        return LIDAR_READER_OK;
    }

private:
    lidar_reader_config_v1 c;
    std::vector<uint8_t> buffer;
    uint64_t byte_count = 0;
    int64_t last_arrival = -1, packet_arrival = -1;
    bool clock_ready = false, raw_ready = false, previous_packet = false;
    uint64_t raw_tick = 0, unwrapped = 0, anchor_tick = 0, previous_tick = 0;
    int64_t anchor_host = 0;
    double previous_angle = 0, previous_step = 0;
    uint64_t previous_id = 0;
    bool previous_point = false, entry_bracketed = false;
    double phase = 0, previous_raw_angle = 0, active_turn = 0;
    int64_t previous_time = 0, last_progress = 0;
    std::vector<lidar_reader_point_v1> active;

    void error(const char *message) {
        std::strncpy(status.last_error, message, sizeof(status.last_error) - 1);
        status.last_error[sizeof(status.last_error) - 1] = 0;
    }

    void reset_sector() {
        previous_point = false; entry_bracketed = false; active.clear();
        phase = previous_raw_angle = active_turn = 0;
        previous_time = last_progress = 0;
    }

    void reset_segment() { previous_packet = false; reset_sector(); }

    std::vector<Event> parse() {
        std::vector<Event> records;
        const uint64_t base = byte_count - buffer.size();
        size_t cursor = 0;
        while (cursor < buffer.size()) {
            if (buffer[cursor] != 0x54) { ++cursor; continue; }
            const size_t remaining = buffer.size() - cursor;
            if (remaining < 3) break;
            const auto *b = buffer.data() + cursor;
            unsigned count = 0;
            size_t size = 0;
            bool short_status = false;
            const unsigned mode = b[1];
            if (mode == 0xa1) {
                const size_t ordinary = 4 + b[2];
                if (ordinary <= 5 && remaining >= ordinary && crc(b, ordinary - 1) == b[ordinary - 1])
                    size = ordinary;
                else if (remaining < 5) break;
                else if (crc(b, 4) == b[4]) { size = 5; short_status = true; }
                else size = ordinary;
            } else if (mode == 0x1f) { size = 6; short_status = true; }
            else if ((mode & 0xe0) == 0x20 && (mode & 0x1f)) {
                count = mode & 0x1f; size = 11 + 3 * count;
            } else if (mode >= 0xa0 && mode <= 0xbf) size = 4 + b[2];
            else { ++cursor; continue; }
            if (remaining < size) break;
            Event event{}; event.offset = base + cursor;
            if (crc(b, size - 1) != b[size - 1]) {
                event.kind = Event::Crc; records.push_back(event); ++cursor; continue;
            }
            event.kind = short_status ? Event::Status : (count ? Event::Scan : Event::Ack);
            if (count) {
                const size_t tail = 6 + 3 * count;
                event.count = count; event.first = le16(b + 4) / 100.;
                event.last = le16(b + tail) / 100.; event.tick = le16(b + tail + 2);
                for (unsigned j = 0; j < count; ++j) {
                    event.distances[j] = le16(b + 6 + 3 * j);
                    event.intensities[j] = b[8 + 3 * j];
                }
            }
            records.push_back(event); cursor += size;
        }
        buffer.erase(buffer.begin(), buffer.begin() + cursor);
        return records;
    }

    void fields(const Event &event) const {
        if (event.tick >= c.timestamp_modulus)
            throw TimingError("raw device tick outside configured modulus");
        if (!event.count || event.count > c.max_points_per_packet)
            throw TimingError("packet point count outside bounded protocol profile");
        if (!(event.first >= 0 && event.first < 360 && event.last >= 0 && event.last < 360))
            throw TimingError("invalid packet angle anchors");
    }

    uint64_t delta_tick(uint64_t next, uint64_t previous) const {
        return next >= previous ? next - previous : c.timestamp_modulus - previous + next;
    }

    void prime(const std::vector<Event> &records, int64_t arrival) {
        size_t count = 0;
        uint64_t first = 0, last = 0, raw = 0;
        for (const auto &event : records) if (event.kind == Event::Scan) {
            fields(event);
            if (!count) first = last = event.tick;
            else {
                const auto delta = delta_tick(event.tick, raw);
                if (!delta || delta > c.max_counter_gap_ticks)
                    throw TimingError("first RX batch has ambiguous counter epoch");
                last += delta;
            }
            raw = event.tick; ++count;
        }
        if (!count) return;
        if ((last - first) * c.nominal_tick_ns > c.max_batch_span_ns)
            throw TimingError("initial receive batch spans too much acquisition history");
        if (count > 4096)
            throw TimingError("prime once using a bounded nonempty first batch");
        clock_ready = true; anchor_tick = last; anchor_host = arrival;
        reset_sector();
    }

    int64_t mapped_offset(double delta) const {
        const double rounded = round_even(delta * c.nominal_tick_ns);
        if (!std::isfinite(rounded) || rounded < -9223372036854775808.0 || rounded >= 9223372036854775808.0)
            throw TimestampValueError{};
        const __int128 result = __int128(anchor_host) + static_cast<int64_t>(rounded);
        if (result < 0 || result > std::numeric_limits<int64_t>::max())
            throw TimestampValueError{};
        return static_cast<int64_t>(result);
    }

    size_t timed_points(const Event &event, int64_t arrival,
                       std::array<lidar_reader_point_v1, 31> &points) {
        fields(event);
        if (packet_arrival >= 0 && (arrival < packet_arrival ||
                uint64_t(arrival - packet_arrival) > c.max_receive_gap_ns))
            throw TimingError("receive discontinuity: new source epoch required", true);
        uint64_t tick = event.tick;
        if (raw_ready) {
            const auto delta = delta_tick(event.tick, raw_tick);
            if (!delta || delta > c.max_counter_gap_ticks)
                throw TimingError("reset/repeated tick/ambiguous wrap: new source epoch required", true);
            tick = unwrapped + delta;
        }
        raw_ready = true; raw_tick = event.tick; unwrapped = tick; packet_arrival = arrival;
        const double span = mod(c.direction * (event.last - event.first), 360);
        if (span >= 180) throw TimingError("packet angle span/direction outside local interpolation model");
        const double step = event.count > 1 ? span / (event.count - 1) : 0;
        const bool have_previous = previous_packet;
        const uint64_t prev_tick = previous_tick, prev_id = previous_id;
        const double prev_angle = previous_angle, prev_step = previous_step;
        previous_packet = true; previous_tick = tick; previous_id = event.offset;
        previous_angle = event.last; previous_step = step;
        if (!have_previous) return 0;
        const double gap = mod(c.direction * (event.first - prev_angle), 360);
        const double limit = c.max_interpacket_step_factor * std::max(step, prev_step) +
                             c.angle_gap_quantization_margin_deg;
        if (event.offset == prev_id || tick - prev_tick > c.max_packet_tick_delta ||
                gap > c.max_raw_angle_gap_deg || gap > limit)
            throw TimingError("missing/repeated packet or angle gap; reset open sector, do not stretch time");
        const double delta = mod(event.last - event.first + 540, 360) - 180;
        const double sample_step = delta / std::max(event.count - 1, 1u);
        int64_t prev_ns = mapped_offset(double(__int128(prev_tick) - anchor_tick));
        for (unsigned j = 0; j < event.count; ++j) {
            const double device_time = double(prev_tick) + double((j + 1) * (tick - prev_tick)) / event.count;
            auto &point = points[j];
            point.time_ns = mapped_offset(device_time - double(anchor_tick));
            if (point.time_ns <= prev_ns)
                throw TimingError("clock scale cannot resolve ordered point times");
            prev_ns = point.time_ns;
            point.packet_offset = event.offset; point.point_index = j;
            point.raw_angle_deg = mod(event.first + j * sample_step, 360);
            const double bearing = c.bearing_sign * (point.raw_angle_deg * (3.14159265358979323846 / 180.)) + c.bearing_zero_rad;
            const double radius = event.distances[j] * .001;
            point.x = radius * std::cos(bearing); point.y = radius * std::sin(bearing);
            point.intensity = event.intensities[j]; point.valid_return = event.distances[j] > 0;
        }
        return event.count;
    }

    void sector(const std::array<lidar_reader_point_v1, 31> &points, size_t count,
                std::vector<std::vector<lidar_reader_point_v1>> &ready) {
        for (size_t i = 0; i < count; ++i) {
            const auto &point = points[i];
            double next_phase;
            if (!previous_point) {
                next_phase = mod(c.direction * point.raw_angle_deg, 360); last_progress = point.time_ns;
            } else {
                const int64_t dt = point.time_ns - previous_time;
                const double gap = mod(c.direction * (point.raw_angle_deg - previous_raw_angle), 360);
                if (dt <= 0 || uint64_t(dt) > c.max_point_gap_ns || gap > c.max_angle_step_deg)
                    throw TimingError("point time/phase gap or reversal; discard sector");
                next_phase = phase + gap;
                if (gap > 1e-9) last_progress = point.time_ns;
                else if (uint64_t(point.time_ns - last_progress) > c.max_stall_ns)
                    throw TimingError("physical scan stalled");
            }
            if (!active.empty()) {
                if (uint64_t(point.time_ns - active.front().time_ns) > c.max_span_ns)
                    throw TimingError("sector acquisition span exceeded");
                if (next_phase >= c.entry + active_turn * 360 + c.width - 1e-9) {
                    if (entry_bracketed) ready.push_back(active);
                    active.clear();
                }
            }
            double turns = (next_phase - c.entry) / 360;
            const double nearest = round_even(turns);
            if (std::abs(turns - nearest) < 1e-12) turns = nearest;
            const double turn = std::floor(turns), entrance = c.entry + turn * 360;
            if (next_phase >= entrance - 1e-9 && next_phase < entrance + c.width - 1e-9) {
                if (active.empty()) {
                    active_turn = turn;
                    entry_bracketed = previous_point && phase < entrance - 1e-9;
                }
                if (active.size() >= c.max_points) throw TimingError("sector point resource bound exceeded");
                active.push_back(point);
            }
            previous_point = true; previous_time = point.time_ns;
            previous_raw_angle = point.raw_angle_deg; phase = next_phase;
        }
    }
};

bool valid(const lidar_reader_config_v1 &c) {
    for (const double value : {c.bearing_sign, c.bearing_zero_rad, c.direction, c.entry, c.width,
            c.nominal_tick_ns, c.max_raw_angle_gap_deg, c.max_interpacket_step_factor,
            c.angle_gap_quantization_margin_deg, c.max_angle_step_deg})
        if (!std::isfinite(value)) return false;
    if ((c.bearing_sign != 1 && c.bearing_sign != -1) || (c.direction != 1 && c.direction != -1) ||
            c.width <= 0 || c.width >= 360 || c.nominal_tick_ns <= 0 ||
            c.timestamp_modulus < 2 || !c.max_counter_gap_ticks ||
            c.max_counter_gap_ticks >= c.timestamp_modulus / 2 ||
            !c.max_packet_tick_delta || c.max_packet_tick_delta > c.max_counter_gap_ticks ||
            !c.max_points_per_packet || c.max_points_per_packet > 31 ||
            !c.max_rx_chunk_bytes || c.max_rx_chunk_bytes > 1048576 ||
            c.max_buffer_bytes < c.max_rx_chunk_bytes || c.max_buffer_bytes > 2097152 ||
            !c.max_points || c.max_points > 20000 || !c.max_batch_span_ns ||
            !c.max_receive_gap_ns || !c.max_span_ns || !c.max_point_gap_ns || !c.max_stall_ns ||
            c.max_angle_step_deg <= 0 || c.max_angle_step_deg >= 180 ||
            c.max_raw_angle_gap_deg <= 0 || c.max_raw_angle_gap_deg >= 180 ||
            c.max_interpacket_step_factor < 1 || c.max_interpacket_step_factor > 10 ||
            c.angle_gap_quantization_margin_deg < 0 || c.angle_gap_quantization_margin_deg >= 1) return false;
    return true;
}
}  // namespace

extern "C" uint32_t lidar_reader_abi_version(void) { return 1; }

extern "C" void *lidar_reader_create_v1(const lidar_reader_config_v1 *config) {
    try { return config && valid(*config) ? new Reader(*config) : nullptr; }
    catch (...) { return nullptr; }
}

extern "C" void lidar_reader_destroy_v1(void *reader) { delete static_cast<Reader *>(reader); }

extern "C" int lidar_reader_feed_v1(void *reader, const uint8_t *data, size_t size,
        int64_t arrival, lidar_reader_scan_v1 callback, void *user) {
    if (!reader || (!data && size) || arrival < 0 || !callback) return LIDAR_READER_INTERNAL;
    try { return static_cast<Reader *>(reader)->feed(data, size, arrival, callback, user); }
    catch (const TimestampValueError &) { return LIDAR_READER_TIMESTAMP_VALUE; }
    catch (const std::exception &e) {
        static_cast<Reader *>(reader)->fault(e.what()); return LIDAR_READER_INTERNAL;
    } catch (...) {
        static_cast<Reader *>(reader)->fault("native lidar reader exception"); return LIDAR_READER_INTERNAL;
    }
}

extern "C" int lidar_reader_get_status_v1(void *reader, lidar_reader_status_v1 *status) {
    if (!reader || !status) return 0;
    *status = static_cast<Reader *>(reader)->status;
    return 1;
}
