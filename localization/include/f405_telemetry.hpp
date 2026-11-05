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
    F405MixedFrames(Callback legacy, Callback extended);
    void feed(const std::uint8_t *, std::size_t);
private:
    Callback legacy_, extended_;
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
} // namespace omni
