#include "scan_protocol.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace t265_map {
namespace protocol {
namespace {

std::uint16_t read_u16_be(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

void write_u16_be(std::uint8_t *data, std::uint16_t value)
{
    data[0] = static_cast<std::uint8_t>((value >> 8) & 0xFFu);
    data[1] = static_cast<std::uint8_t>(value & 0xFFu);
}

bool accepted_type(std::uint8_t type)
{
    return type == kMsgOdom || type == kMsgLegacyStmStatus ||
           type == kMsgScanMotionCommand || type == kMsgScanMotionStatus;
}

}  // namespace

std::uint16_t crc16_modbus(const std::uint8_t *data, std::size_t size)
{
    std::uint16_t crc = 0xFFFFu;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) != 0u
                ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001u)
                : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

std::array<std::uint8_t, kFrameSize> build_motion_command(
    std::uint8_t sequence, const MotionCommandPayload &payload)
{
    std::array<std::uint8_t, kFrameSize> frame{};
    frame[0] = kFrameHead1;
    frame[1] = kFrameHead2;
    frame[2] = kMsgScanMotionCommand;
    frame[3] = sequence;
    frame[4] = payload.command;
    frame[5] = payload.flags;
    write_u16_be(frame.data() + 6,
                 static_cast<std::uint16_t>(payload.arg1));
    write_u16_be(frame.data() + 8,
                 static_cast<std::uint16_t>(payload.arg2));
    write_u16_be(frame.data() + 10, payload.speed);
    const std::uint16_t crc = crc16_modbus(frame.data() + 2, 10);
    // The shared UART framing is CRC_LO, CRC_HI (little-endian), unlike the
    // big-endian motion arguments above.
    frame[12] = static_cast<std::uint8_t>(crc & 0xFFu);
    frame[13] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    frame[14] = kFrameTail;
    return frame;
}

bool parse_frame(const std::uint8_t *frame, std::size_t size,
                 std::uint8_t &message_type, std::uint8_t &sequence,
                 std::array<std::uint8_t, kPayloadSize> &payload)
{
    if (frame == nullptr || size != kFrameSize ||
        frame[0] != kFrameHead1 || frame[1] != kFrameHead2 ||
        frame[14] != kFrameTail) {
        return false;
    }
    if (!accepted_type(frame[2])) return false;
    const std::uint16_t expected = crc16_modbus(frame + 2, 10);
    const std::uint16_t actual = static_cast<std::uint16_t>(frame[12]) |
                                 static_cast<std::uint16_t>(frame[13] << 8);
    if (expected != actual) return false;
    message_type = frame[2];
    sequence = frame[3];
    std::copy(frame + 4, frame + 12, payload.begin());
    return true;
}

bool decode_odom(std::uint8_t sequence,
                 const std::array<std::uint8_t, kPayloadSize> &payload,
                 OdomPayload &odom)
{
    odom.sequence = sequence;
    odom.m1_count = read_u16_be(payload.data() + 0);
    odom.m2_count = read_u16_be(payload.data() + 2);
    odom.m3_count = read_u16_be(payload.data() + 4);
    odom.dt_ms = payload[6];
    odom.status = payload[7];
    return odom.dt_ms != 0;
}

bool decode_motion_status(
    std::uint8_t sequence,
    const std::array<std::uint8_t, kPayloadSize> &payload,
    MotionStatusPayload &status)
{
    status.frame_sequence = sequence;
    status.acknowledged_command_sequence = payload[0];
    status.state = payload[1];
    status.fault = payload[2];
    status.command = payload[3];
    status.progress = static_cast<std::int16_t>(read_u16_be(payload.data() + 4));
    status.heading_cdeg = read_u16_be(payload.data() + 6);
    return status.state <= kMotionStopped && status.heading_cdeg < 36000u;
}

const char *motion_command_name(std::uint8_t command)
{
    switch (command) {
        case kMotionStop: return "STOP";
        case kMotionHold: return "HOLD";
        case kMotionTurnRelative: return "TURN_REL";
        case kMotionMoveBody: return "MOVE_BODY";
        case kMotionMoveField: return "MOVE_FIELD";
        case kMotionResetOdom: return "RESET_ODOM";
        default: return "UNKNOWN";
    }
}

const char *motion_state_name(std::uint8_t state)
{
    switch (state) {
        case kMotionIdle: return "IDLE";
        case kMotionRunning: return "RUNNING";
        case kMotionDone: return "DONE";
        case kMotionError: return "ERROR";
        case kMotionStopped: return "STOPPED";
        default: return "UNKNOWN";
    }
}

StreamParser::StreamParser(FrameCallback callback)
    : callback_(std::move(callback))
{
}

void StreamParser::reset()
{
    buffered_ = 0;
}

void StreamParser::feed(const std::uint8_t *data, std::size_t size)
{
    if (data == nullptr || size == 0) return;
    bytes_ += size;
    std::size_t consumed = 0;
    while (consumed < size) {
        if (buffered_ == buffer_.size()) {
            buffered_ = 0;
            ++malformed_frames_;
        }
        const std::size_t room = buffer_.size() - buffered_;
        const std::size_t copy_size = std::min(room, size - consumed);
        std::copy(data + consumed, data + consumed + copy_size,
                  buffer_.begin() + static_cast<std::ptrdiff_t>(buffered_));
        buffered_ += copy_size;
        consumed += copy_size;

        while (buffered_ >= kFrameSize) {
            std::size_t start = 0;
            while (start + 1 < buffered_ &&
                   !(buffer_[start] == kFrameHead1 &&
                     buffer_[start + 1] == kFrameHead2)) {
                ++start;
            }
            if (start + 1 >= buffered_) {
                // Keep a possible first header byte for the next chunk.
                if (buffered_ > 0 && buffer_[buffered_ - 1] == kFrameHead1) {
                    buffer_[0] = kFrameHead1;
                    buffered_ = 1;
                } else {
                    buffered_ = 0;
                }
                break;
            }
            if (start > 0) {
                std::memmove(buffer_.data(), buffer_.data() + start,
                             buffered_ - start);
                buffered_ -= start;
            }
            if (buffered_ < kFrameSize) break;

            std::uint8_t type = 0;
            std::uint8_t sequence = 0;
            std::array<std::uint8_t, kPayloadSize> payload{};
            if (!parse_frame(buffer_.data(), kFrameSize, type, sequence, payload)) {
                const std::uint16_t expected = crc16_modbus(buffer_.data() + 2, 10);
                const std::uint16_t actual = static_cast<std::uint16_t>(buffer_[12]) |
                    static_cast<std::uint16_t>(buffer_[13] << 8);
                if (expected != actual && accepted_type(buffer_[2]) &&
                    buffer_[0] == kFrameHead1 && buffer_[1] == kFrameHead2 &&
                    buffer_[14] == kFrameTail) {
                    ++crc_errors_;
                } else {
                    ++malformed_frames_;
                }
                std::memmove(buffer_.data(), buffer_.data() + 1, buffered_ - 1);
                --buffered_;
                continue;
            }
            std::memmove(buffer_.data(), buffer_.data() + kFrameSize,
                         buffered_ - kFrameSize);
            buffered_ -= kFrameSize;
            ++frames_;
            if (callback_) callback_(type, sequence, payload);
        }
    }
}

}  // namespace protocol
}  // namespace t265_map
