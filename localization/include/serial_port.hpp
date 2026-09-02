#ifndef T265_OMNI_SERIAL_PORT_HPP
#define T265_OMNI_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace omni {

class SerialPort {
public:
    SerialPort(std::string path, int baud);
    ~SerialPort();
    SerialPort(const SerialPort &) = delete;
    SerialPort &operator=(const SerialPort &) = delete;

    void open_port();
    void close_port();
    int read_some(std::uint8_t *data, std::size_t capacity, int timeout_ms);
    bool write_all(const std::uint8_t *data, std::size_t size, int timeout_ms);
    const std::string &path() const noexcept { return path_; }

private:
    std::string path_;
    int baud_;
    int descriptor_ = -1;
};

}  // namespace omni

#endif
