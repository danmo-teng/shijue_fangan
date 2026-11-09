#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace omni {
// Split complete wire containers before the existing mission/status parser.
// No clock/age qualification: extended payload may itself contain A3 B3.
class F405MixedFrames {
public:
    using Callback = std::function<void(const std::uint8_t *, std::size_t)>;
    F405MixedFrames(Callback legacy, Callback extended, bool all_legacy=false);
    void feed(const std::uint8_t *, std::size_t);
private:
    Callback legacy_, extended_;
    bool all_legacy_;
    std::vector<std::uint8_t> buffer_;
};

// One-way measured-feedback tap. C++ remains the sole physical UART owner.
class WheelTelemetryTap {
public:
    explicit WheelTelemetryTap(const std::string &address);
    ~WheelTelemetryTap();
    void publish(const std::uint8_t *, std::size_t, std::uint64_t received_ns);
private:
    int fd_ = -1;
    std::string address_;
};

struct MotionTelemetryStats {
    std::uint64_t eligible_frames = 0;
    std::uint64_t sent_frames = 0;
    std::uint64_t dropped_frames = 0;
    std::uint64_t filtered_frames = 0;
    std::uint64_t send_errors = 0;
    int connection_error = 0;
    int last_error = 0;
    bool connected = false;
};

// Independent control-input lane. publish receives only CRC-validated complete
// extended frames from F405MixedFrames and never sends a request to the MCU.
// One UART publisher owns these counters; inspect stats after it has stopped.
class MotionTelemetryTap {
public:
    explicit MotionTelemetryTap(const std::string &address);
    ~MotionTelemetryTap();
    MotionTelemetryTap(const MotionTelemetryTap &) = delete;
    MotionTelemetryTap &operator=(const MotionTelemetryTap &) = delete;
    void publish(const std::uint8_t *, std::size_t, std::uint64_t received_ns);
    const MotionTelemetryStats &stats() const { return stats_; }
private:
    int fd_ = -1;
    bool enabled_ = false;
    MotionTelemetryStats stats_;
};
} // namespace omni
