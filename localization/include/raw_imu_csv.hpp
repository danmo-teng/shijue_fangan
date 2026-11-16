#pragma once

#include "async_csv_log.hpp"
#include <cstdio>

namespace omni {
// Raw sensor axes and SDK capture clock only. No estimator, host clock query,
// readiness condition, or synchronous disk write on the SDK callback.
class RawImuCsv {
public:
    explicit RawImuCsv(const std::string &path) : log_(path) {
        log_.submit("sdk_run,stream,frame_number,capture_timestamp_ms,timestamp_domain,x,y,z,temperature_c\n");
    }
    bool enabled() const { return log_.enabled(); }
    void submit(std::uint64_t run, bool gyro, std::uint64_t frame, double stamp,
                int domain, float x, float y, float z, bool have_temperature, double temperature) {
        if (!enabled()) return;
        char row[256], temperature_text[32] = "";
        if (have_temperature) std::snprintf(temperature_text, sizeof(temperature_text), "%.9g", temperature);
        const int n = std::snprintf(row, sizeof(row), "%llu,%s,%llu,%.6f,%d,%.9g,%.9g,%.9g,%s\n",
            static_cast<unsigned long long>(run), gyro ? "gyro" : "accel",
            static_cast<unsigned long long>(frame), stamp, domain, x, y, z, temperature_text);
        if (n < 0 || static_cast<std::size_t>(n) >= sizeof(row)) { ++read_errors_; return; }
        ++samples_;
        log_.submit(std::string(row, static_cast<std::size_t>(n)));
    }
    void read_error() { ++read_errors_; }
    void close() { log_.close(); }
    std::uint64_t samples() const { return samples_; }
    std::uint64_t read_errors() const { return read_errors_; }
    std::uint64_t dropped() const { return log_.dropped(); }
    bool failed() const { return log_.failed(); }
private:
    AsyncCsvLog log_;
    std::atomic<std::uint64_t> samples_{0}, read_errors_{0};
};
} // namespace omni
