#ifndef T265_MAP_SCAN_PROTOCOL_HPP
#define T265_MAP_SCAN_PROTOCOL_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace t265_map {
namespace protocol {

constexpr std::uint8_t kFrameHead1 = 0xA3;
constexpr std::uint8_t kFrameHead2 = 0xB3;
constexpr std::uint8_t kFrameTail = 0xC3;
constexpr std::size_t kFrameSize = 15;
constexpr std::size_t kPayloadSize = 8;

// Existing F407 telemetry retained for comparison in the scanner.
constexpr std::uint8_t kMsgOdom = 0x15;

// These two message types are reserved for the scanner/F407 cooperation.
// They do not overlap the current task's 0x17 STM status and 0x18 mission
// command messages.
constexpr std::uint8_t kMsgScanMotionCommand = 0x19;
constexpr std::uint8_t kMsgScanMotionStatus = 0x1A;

// The current F407 task status is accepted and logged as an opaque frame so
// that a scanner can share the UART during protocol bring-up. It is never
// interpreted as a scan-motion status.
constexpr std::uint8_t kMsgLegacyStmStatus = 0x17;

enum MotionCommand : std::uint8_t {
    kMotionStop = 0,
    kMotionHold = 1,
    kMotionTurnRelative = 2,
    kMotionMoveBody = 3,
    kMotionMoveField = 4,
    kMotionResetOdom = 5,
};

enum MotionFlags : std::uint8_t {
    kFlagValid = 1u << 0,
    kFlagKeepHeading = 1u << 1,
    kFlagFieldFrame = 1u << 2,
    kFlagAckRequired = 1u << 3,
    kFlagClearFault = 1u << 4,
};

enum MotionState : std::uint8_t {
    kMotionIdle = 0,
    kMotionRunning = 1,
    kMotionDone = 2,
    kMotionError = 3,
    kMotionStopped = 4,
};

struct MotionCommandPayload {
    std::uint8_t command = kMotionStop;
    std::uint8_t flags = kFlagValid | kFlagAckRequired;
    // TURN_REL: signed deci-degrees (0.1 degree), positive is
    // counter-clockwise/left. Deci-degrees fit a full 360 degree test in
    // int16_t, unlike centidegrees.
    // MOVE_BODY: signed forward and left millimetres.
    // MOVE_FIELD: signed field +X and +Y millimetres.
    std::int16_t arg1 = 0;
    std::int16_t arg2 = 0;
    // Translation commands: mm/s. TURN_REL: deci-degrees/s.
    std::uint16_t speed = 0;
};

struct OdomPayload {
    std::uint8_t sequence = 0;
    std::uint16_t m1_count = 0;
    std::uint16_t m2_count = 0;
    std::uint16_t m3_count = 0;
    std::uint8_t dt_ms = 0;
    std::uint8_t status = 0;
};

struct MotionStatusPayload {
    std::uint8_t frame_sequence = 0;
    std::uint8_t acknowledged_command_sequence = 0;
    std::uint8_t state = kMotionIdle;
    std::uint8_t fault = 0;
    std::uint8_t command = kMotionStop;
    // MOVE_*: signed progress in millimetres; TURN_REL: signed progress in
    // deci-degrees.
    std::int16_t progress = 0;
    // Current F407 IMU heading, 0..35999 centidegrees.
    std::uint16_t heading_cdeg = 0;
};

std::uint16_t crc16_modbus(const std::uint8_t *data, std::size_t size);

std::array<std::uint8_t, kFrameSize> build_motion_command(
    std::uint8_t sequence, const MotionCommandPayload &payload);

bool parse_frame(const std::uint8_t *frame, std::size_t size,
                 std::uint8_t &message_type, std::uint8_t &sequence,
                 std::array<std::uint8_t, kPayloadSize> &payload);

bool decode_odom(std::uint8_t sequence,
                 const std::array<std::uint8_t, kPayloadSize> &payload,
                 OdomPayload &odom);

bool decode_motion_status(
    std::uint8_t sequence,
    const std::array<std::uint8_t, kPayloadSize> &payload,
    MotionStatusPayload &status);

const char *motion_command_name(std::uint8_t command);
const char *motion_state_name(std::uint8_t state);

class StreamParser {
public:
    using FrameCallback = std::function<void(
        std::uint8_t, std::uint8_t,
        const std::array<std::uint8_t, kPayloadSize> &)>;

    explicit StreamParser(FrameCallback callback = FrameCallback());

    void feed(const std::uint8_t *data, std::size_t size);
    void reset();

    std::uint64_t bytes() const noexcept { return bytes_; }
    std::uint64_t frames() const noexcept { return frames_; }
    std::uint64_t crc_errors() const noexcept { return crc_errors_; }
    std::uint64_t malformed_frames() const noexcept { return malformed_frames_; }

private:
    std::array<std::uint8_t, 512> buffer_{};
    std::size_t buffered_ = 0;
    FrameCallback callback_;
    std::uint64_t bytes_ = 0;
    std::uint64_t frames_ = 0;
    std::uint64_t crc_errors_ = 0;
    std::uint64_t malformed_frames_ = 0;
};

}  // namespace protocol
}  // namespace t265_map

#endif
