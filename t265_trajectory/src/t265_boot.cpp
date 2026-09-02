#include <libusb.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef T265_FIRMWARE_PATH
#error "T265_FIRMWARE_PATH must be defined by CMake"
#endif

namespace {

constexpr std::uint16_t kBootVid = 0x03e7;
constexpr std::uint16_t kBootPid = 0x2150;
constexpr std::uint16_t kRunVid = 0x8087;
constexpr std::uint16_t kRunPid = 0x0b37;

class UsbContext {
public:
    UsbContext()
    {
        const int status = libusb_init(&value_);
        if (status != LIBUSB_SUCCESS) {
            throw std::runtime_error(std::string("libusb_init: ") + libusb_error_name(status));
        }
    }
    ~UsbContext() { libusb_exit(value_); }
    libusb_context *get() const { return value_; }
private:
    libusb_context *value_ = nullptr;
};

class UsbHandle {
public:
    explicit UsbHandle(libusb_device_handle *value) : value_(value) {}
    ~UsbHandle() { if (value_) libusb_close(value_); }
    libusb_device_handle *get() const { return value_; }
private:
    libusb_device_handle *value_;
};

std::vector<unsigned char> read_firmware()
{
    std::ifstream file(T265_FIRMWARE_PATH, std::ios::binary);
    if (!file) {
        throw std::runtime_error(std::string("cannot open firmware: ") + T265_FIRMWARE_PATH);
    }
    return std::vector<unsigned char>(std::istreambuf_iterator<char>(file),
                                      std::istreambuf_iterator<char>());
}

int find_bulk_out_endpoint(libusb_device *device, int &interface_number)
{
    libusb_config_descriptor *config = nullptr;
    int status = libusb_get_active_config_descriptor(device, &config);
    if (status != LIBUSB_SUCCESS) {
        throw std::runtime_error(std::string("get active USB configuration: ") +
                                 libusb_error_name(status));
    }

    int endpoint = -1;
    for (std::uint8_t i = 0; i < config->bNumInterfaces && endpoint < 0; ++i) {
        const libusb_interface &iface = config->interface[i];
        for (int a = 0; a < iface.num_altsetting && endpoint < 0; ++a) {
            const libusb_interface_descriptor &alt = iface.altsetting[a];
            for (std::uint8_t e = 0; e < alt.bNumEndpoints; ++e) {
                const libusb_endpoint_descriptor &ep = alt.endpoint[e];
                const bool is_out = (ep.bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT;
                const bool is_bulk = (ep.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK;
                if (is_out && is_bulk) {
                    endpoint = ep.bEndpointAddress;
                    interface_number = alt.bInterfaceNumber;
                    break;
                }
            }
        }
    }
    libusb_free_config_descriptor(config);
    return endpoint;
}

bool device_present(libusb_context *context, std::uint16_t vid, std::uint16_t pid)
{
    libusb_device **devices = nullptr;
    const ssize_t count = libusb_get_device_list(context, &devices);
    if (count < 0) {
        return false;
    }
    bool found = false;
    for (ssize_t i = 0; i < count; ++i) {
        libusb_device_descriptor descriptor{};
        if (libusb_get_device_descriptor(devices[i], &descriptor) == LIBUSB_SUCCESS &&
            descriptor.idVendor == vid && descriptor.idProduct == pid) {
            found = true;
            break;
        }
    }
    libusb_free_device_list(devices, 1);
    return found;
}

struct Options {
    unsigned timeout_ms = 15000;
    unsigned chunk_kib = 256;
    bool reset_usb = false;
};

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument(argv[i]);
        if (argument == "-h" || argument == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [--timeout-ms MS] [--chunk-kib KiB] [--reset-usb]\n"
                      << "Load official 0.2.0.951 firmware into a T265 in 03e7:2150 boot state.\n";
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--reset-usb") {
            options.reset_usb = true;
            continue;
        }
        if ((argument != "--timeout-ms" && argument != "--chunk-kib") || i + 1 >= argc) {
            throw std::invalid_argument("unknown or incomplete option: " + argument);
        }
        const std::string value(argv[++i]);
        std::size_t used = 0;
        const unsigned long parsed = std::stoul(value, &used);
        if (used != value.size() || parsed == 0) {
            throw std::invalid_argument("invalid value for " + argument + ": " + value);
        }
        if (argument == "--timeout-ms") {
            if (parsed > 120000) {
                throw std::invalid_argument("invalid --timeout-ms: " + value);
            }
            options.timeout_ms = static_cast<unsigned>(parsed);
        } else {
            if (parsed > 4096) {
                throw std::invalid_argument("invalid --chunk-kib: " + value);
            }
            options.chunk_kib = static_cast<unsigned>(parsed);
        }
    }
    return options;
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        const Options options = parse_options(argc, argv);
        UsbContext context;
        if (device_present(context.get(), kRunVid, kRunPid)) {
            std::cout << "[READY] T265 is already running as 8087:0b37\n";
            return EXIT_SUCCESS;
        }

        UsbHandle handle(libusb_open_device_with_vid_pid(context.get(), kBootVid, kBootPid));
        if (!handle.get()) {
            std::cerr << "[ERROR] no T265 boot device 03e7:2150 found or it cannot be opened\n";
            return 2;
        }

        if (options.reset_usb) {
            std::cout << "[RESET] resetting only USB device 03e7:2150 before transfer\n";
            const int reset_status = libusb_reset_device(handle.get());
            if (reset_status != LIBUSB_SUCCESS) {
                throw std::runtime_error(std::string("USB reset failed: ") +
                                         libusb_error_name(reset_status));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        int interface_number = -1;
        const int endpoint = find_bulk_out_endpoint(libusb_get_device(handle.get()), interface_number);
        if (endpoint < 0) {
            throw std::runtime_error("no bulk OUT endpoint found on T265 boot device");
        }

        libusb_set_auto_detach_kernel_driver(handle.get(), 1);
        int status = libusb_claim_interface(handle.get(), interface_number);
        if (status != LIBUSB_SUCCESS) {
            throw std::runtime_error(std::string("claim interface: ") + libusb_error_name(status));
        }

        const std::vector<unsigned char> firmware = read_firmware();
        std::cout << "[BOOT] firmware=" << T265_FIRMWARE_PATH << '\n'
                  << "[BOOT] bytes=" << firmware.size() << " endpoint=0x"
                  << std::hex << endpoint << std::dec << " interface=" << interface_number
                  << " timeout_per_chunk=" << options.timeout_ms << " ms"
                  << " chunk=" << options.chunk_kib << " KiB\n";

        const auto start = std::chrono::steady_clock::now();
        std::size_t total_transferred = 0;
        const std::size_t chunk_bytes = static_cast<std::size_t>(options.chunk_kib) * 1024u;
        while (total_transferred < firmware.size()) {
            const std::size_t request_size = std::min(chunk_bytes,
                                                       firmware.size() - total_transferred);
            int transferred = 0;
            status = libusb_bulk_transfer(
                handle.get(), static_cast<unsigned char>(endpoint),
                const_cast<unsigned char *>(firmware.data() + total_transferred),
                static_cast<int>(request_size), &transferred, options.timeout_ms);
            if (transferred > 0) {
                total_transferred += static_cast<std::size_t>(transferred);
                std::cout << "\r[BOOT] progress=" << total_transferred << '/' << firmware.size()
                          << " (" << (100u * total_transferred / firmware.size()) << "%)"
                          << std::flush;
            }
            if (status != LIBUSB_SUCCESS || transferred != static_cast<int>(request_size)) {
                break;
            }
        }
        const double seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        if (total_transferred > 0) {
            std::cout << '\n';
        }

        if (status != LIBUSB_SUCCESS) {
            std::cerr << "[ERROR] firmware transfer failed: " << libusb_error_name(status)
                      << " (" << libusb_strerror(static_cast<libusb_error>(status)) << ")"
                      << ", transferred=" << total_transferred << '/' << firmware.size()
                      << ", elapsed=" << std::fixed << std::setprecision(3) << seconds << " s\n";
            libusb_release_interface(handle.get(), interface_number);
            return 3;
        }
        if (total_transferred != firmware.size()) {
            std::cerr << "[ERROR] short firmware transfer: " << total_transferred << '/'
                      << firmware.size() << " bytes\n";
            libusb_release_interface(handle.get(), interface_number);
            return 4;
        }

        std::cout << "[BOOT] transfer complete in " << std::fixed << std::setprecision(3)
                  << seconds << " s; waiting for 8087:0b37...\n";
        libusb_release_interface(handle.get(), interface_number);

        // The same handle remains scoped here, but libusb enumeration uses a new
        // device list and observes the T265 after its firmware-triggered reconnect.
        for (int i = 0; i < 50; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (device_present(context.get(), kRunVid, kRunPid)) {
                std::cout << "[READY] T265 re-enumerated as 8087:0b37\n";
                return EXIT_SUCCESS;
            }
        }
        std::cerr << "[ERROR] firmware transfer succeeded, but 8087:0b37 did not appear within 10 s\n";
        return 5;
    } catch (const std::exception &error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
