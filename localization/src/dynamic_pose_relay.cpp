#include "dynamic_pose_relay.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace omni {
DynamicPoseRelay::DynamicPoseRelay(const std::string &address, const std::string &session,
                                 const std::string &output, Send send)
    : session_(session), output_(output), send_(std::move(send))
{
    if (address.empty()) return; // Offline fixture.
    sockaddr_un a{}; a.sun_family = AF_UNIX;
    if (address[0] != '@' || address.size() >= sizeof(a.sun_path))
        throw std::invalid_argument("dynamic pose requires short @abstract socket");
    std::memcpy(a.sun_path + 1, address.data() + 1, address.size() - 1);
    fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0 || ::bind(fd_, reinterpret_cast<sockaddr *>(&a),
                        offsetof(sockaddr_un, sun_path) + address.size()) != 0) {
        if (fd_ >= 0) ::close(fd_);
        throw std::runtime_error("cannot bind dynamic pose relay");
    }
}
DynamicPoseRelay::~DynamicPoseRelay() { stop(); if (fd_ >= 0) ::close(fd_); }
void DynamicPoseRelay::start() { thread_ = std::thread(&DynamicPoseRelay::run, this); }
void DynamicPoseRelay::stop() { stop_ = true; if (thread_.joinable()) thread_.join(); }

FusedPoseFrame DynamicPoseRelay::frame(const Json &p, std::uint8_t sequence)
{
    const auto &pose = p.at("pose");
    const double x = pose.at("x_m").get<double>(), y = pose.at("y_m").get<double>();
    const double yaw = pose.at("yaw_deg").get<double>();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        std::fabs(x) > 32.767 || std::fabs(y) > 32.767)
        throw std::invalid_argument("unrepresentable dynamic pose");
    FusedPoseFrame f; f.sequence = sequence;
    f.x_mm = static_cast<std::int16_t>(std::lround(x * 1000));
    f.y_mm = static_cast<std::int16_t>(std::lround(y * 1000));
    f.heading_cdeg = static_cast<std::uint16_t>(std::lround(
        std::fmod(std::fmod(yaw, 360.) + 360., 360.) * 100)) % 36000u;
    const auto quality = p.at("quality").get<std::string>();
    const bool valid = quality == "GOOD" || quality == "DEGRADED";
    const auto mode = p.at("motion_source").get<std::string>();
    const unsigned confidence = p.at("t265").at("tracker_confidence").get<unsigned>();
    if (valid) f.status |= kPoseValid;
    if (valid && mode != "WHEEL_FALLBACK" && confidence >= 2) f.status |= kPoseT265Good;
    if (valid && (mode == "WHEEL_FALLBACK" || mode == "T265_WITH_WHEEL_TAIL")) f.status |= kPoseWheelActive;
    if (std::fabs(x) <= 1.5 && std::fabs(y) <= 1.5) f.status |= kPoseInsideField;
    // Unknown position error: use the saturated legacy sigma nibble, not a
    // fabricated small covariance. Raw tracker confidence remains raw.
    f.confidence_and_sigma = static_cast<std::uint8_t>(0xf0u | ((p.at("t265").value("mapper_confidence",0u) & 3u) << 2) | (confidence & 3u));
    return f;
}

bool DynamicPoseRelay::accept(Json packet)
{
    if (packet.at("schema_version") != 1 || packet.at("session_id") != session_ ||
        packet.at("frame") != "field") return false;
    const auto id = packet.at("packet_id").get<std::uint64_t>();
    if (have_packet_ && id <= packet_id_) return false;
    const auto bytes = build_fused_pose_frame(frame(packet, sequence_));
    have_packet_ = true; packet_id_ = id; ++sequence_;
    // Same snapshot feeds UART and formal JSON, independently of SDK Pose arrival.
    const bool sent = send_(bytes);
    if (sent) ++tx_ok_; else ++tx_error_;
    packet["control_pose_published"] = true;
    packet["uart"] = {{"pose_tx_frames", tx_ok_}, {"pose_tx_errors", tx_error_},
                      {"last_pose_write_success", sent}};
    packet["json_write_errors"] = json_errors_;
    const std::string temporary = output_ + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    file << packet.dump() << '\n'; file.close();
    if (!file || std::rename(temporary.c_str(), output_.c_str()) != 0) {
        if (++json_errors_ == 1) std::cerr << "[DYNAMIC POSE] JSON write failed; UART relay continues\n";
    }
    return true;
}

void DynamicPoseRelay::run()
{
    while (!stop_) {
        pollfd p{fd_, POLLIN, 0};
        if (::poll(&p, 1, 50) <= 0) continue;
        // Coalesce bounded snapshots, never replay a backlog to the chassis.
        std::string latest; char buffer[32768];
        for (int i = 0; i < 32; ++i) {
            const auto n = ::recv(fd_, buffer, sizeof(buffer), MSG_DONTWAIT | MSG_TRUNC);
            if (n < 0) break;
            if (n > 0 && n < static_cast<ssize_t>(sizeof(buffer))) latest.assign(buffer, n);
        }
        if (latest.empty()) continue;
        try { accept(Json::parse(latest)); }
        catch (const std::exception &e) { std::cerr << "[DYNAMIC POSE] invalid packet: " << e.what() << '\n'; }
    }
}
}
