#ifndef T265_OMNI_F407_PROTOCOL_HPP
#define T265_OMNI_F407_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace omni {

constexpr std::uint8_t kFrameHead1 = 0xA3;
constexpr std::uint8_t kFrameHead2 = 0xB3;
constexpr std::uint8_t kFrameTail = 0xC3;
constexpr std::uint8_t kOdomMessageType = 0x15;
constexpr std::uint8_t kFusedPoseMessageType = 0x16;
constexpr std::uint8_t kStmStatusMessageType = 0x17;
constexpr std::uint8_t kMissionCommandMessageType = 0x18;
constexpr std::size_t kFrameSize = 15;

constexpr std::uint8_t kOdomM1Valid = 1u << 0;
constexpr std::uint8_t kOdomM2Valid = 1u << 1;
constexpr std::uint8_t kOdomM3Valid = 1u << 2;
constexpr std::uint8_t kOdomCounterReset = 1u << 3;
constexpr std::uint8_t kOdomEncoderFault = 1u << 4;

constexpr std::uint8_t kPoseValid = 1u << 0;
constexpr std::uint8_t kPoseT265Good = 1u << 1;
constexpr std::uint8_t kPoseWheelActive = 1u << 2;
constexpr std::uint8_t kPoseObstacleGate = 1u << 3;
constexpr std::uint8_t kPoseOdomFresh = 1u << 4;
constexpr std::uint8_t kPoseInsideField = 1u << 5;
constexpr std::uint8_t kPoseT265UpdateRejected = 1u << 6;

struct EncoderFrame {
    std::uint8_t sequence = 0;
    std::uint16_t position[3] = {0, 0, 0};
    std::uint8_t sample_period_ms = 0;
    std::uint8_t status = 0;
};

struct FusedPoseFrame {
    std::uint8_t sequence = 0;
    std::int16_t x_mm = 0;
    std::int16_t y_mm = 0;
    std::uint16_t heading_cdeg = 0;
    std::uint8_t status = 0;
    std::uint8_t confidence_and_sigma = 0;
};

struct StmStatusFrame {
    std::uint8_t sequence = 0;
    std::uint8_t flags = 0;
    std::uint8_t mode = 0;
    std::uint16_t camera_pitch_cdeg = 0;
    std::uint8_t acknowledged_sequence = 0;
    std::uint8_t fault_code = 0;
};

struct ParserStats {
    std::uint64_t bytes = 0;
    std::uint64_t frames_ok = 0;
    std::uint64_t crc_errors = 0;
    std::uint64_t malformed = 0;
    std::uint64_t sequence_gaps = 0;
    std::uint64_t duplicates = 0;
    std::uint64_t status_frames = 0;
};

std::uint16_t modbus_crc16(const std::uint8_t *data, std::size_t size);
std::array<std::uint8_t, kFrameSize> build_encoder_frame(const EncoderFrame &frame);
std::array<std::uint8_t, kFrameSize> build_fused_pose_frame(const FusedPoseFrame &frame);
std::array<std::uint8_t, kFrameSize> build_stm_status_frame(const StmStatusFrame &frame);
bool decode_fused_pose_frame(const std::uint8_t *data, std::size_t size,
                             FusedPoseFrame &frame);
bool validate_relay_frame(const std::uint8_t *data, std::size_t size);

class F407FrameParser {
public:
    using Callback = std::function<void(const EncoderFrame &)>;
    using StatusCallback = std::function<void(const StmStatusFrame &)>;

    explicit F407FrameParser(Callback callback,
                             StatusCallback status_callback = StatusCallback{});
    void feed(const std::uint8_t *data, std::size_t size);
    const ParserStats &stats() const noexcept { return stats_; }

private:
    void consume(std::uint8_t value);
    void validate_frame();
    void resynchronize();

    Callback callback_;
    StatusCallback status_callback_;
    std::array<std::uint8_t, kFrameSize> frame_{};
    std::size_t index_ = 0;
    ParserStats stats_{};
    bool have_sequence_ = false;
    std::uint8_t last_sequence_ = 0;
    bool have_status_sequence_ = false;
    std::uint8_t last_status_sequence_ = 0;
};

}  // namespace omni

#endif
