#include "f407_protocol.hpp"

#include <algorithm>

namespace omni {
namespace {

std::uint16_t get_u16_be(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
}

std::int16_t get_i16_be(const std::uint8_t *data)
{
    return static_cast<std::int16_t>(get_u16_be(data));
}

void put_u16_be(std::uint8_t *data, std::uint16_t value)
{
    data[0] = static_cast<std::uint8_t>((value >> 8) & 0xffu);
    data[1] = static_cast<std::uint8_t>(value & 0xffu);
}

}  // namespace

std::uint16_t modbus_crc16(const std::uint8_t *data, std::size_t size)
{
    std::uint16_t crc = 0xffffu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (std::uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xa001u)
                             : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

std::array<std::uint8_t, kFrameSize> build_encoder_frame(const EncoderFrame &input)
{
    std::array<std::uint8_t, kFrameSize> frame{};
    frame[0] = kFrameHead1;
    frame[1] = kFrameHead2;
    frame[2] = kOdomMessageType;
    frame[3] = input.sequence;
    put_u16_be(&frame[4], input.position[0]);
    put_u16_be(&frame[6], input.position[1]);
    put_u16_be(&frame[8], input.position[2]);
    frame[10] = input.sample_period_ms;
    frame[11] = input.status;
    const std::uint16_t crc = modbus_crc16(&frame[2], 10);
    frame[12] = static_cast<std::uint8_t>(crc & 0xffu);
    frame[13] = static_cast<std::uint8_t>((crc >> 8) & 0xffu);
    frame[14] = kFrameTail;
    return frame;
}

std::array<std::uint8_t, kFrameSize> build_fused_pose_frame(const FusedPoseFrame &input)
{
    std::array<std::uint8_t, kFrameSize> frame{};
    frame[0] = kFrameHead1;
    frame[1] = kFrameHead2;
    frame[2] = kFusedPoseMessageType;
    frame[3] = input.sequence;
    put_u16_be(&frame[4], static_cast<std::uint16_t>(input.x_mm));
    put_u16_be(&frame[6], static_cast<std::uint16_t>(input.y_mm));
    put_u16_be(&frame[8], input.heading_cdeg);
    frame[10] = input.status;
    frame[11] = input.confidence_and_sigma;
    const std::uint16_t crc = modbus_crc16(&frame[2], 10);
    frame[12] = static_cast<std::uint8_t>(crc & 0xffu);
    frame[13] = static_cast<std::uint8_t>((crc >> 8) & 0xffu);
    frame[14] = kFrameTail;
    return frame;
}

std::array<std::uint8_t, kFrameSize> build_stm_status_frame(const StmStatusFrame &input)
{
    std::array<std::uint8_t, kFrameSize> frame{};
    frame[0] = kFrameHead1;
    frame[1] = kFrameHead2;
    frame[2] = kStmStatusMessageType;
    frame[3] = input.sequence;
    frame[4] = input.flags;
    frame[5] = input.mode;
    put_u16_be(&frame[6], input.camera_pitch_cdeg);
    frame[8] = input.acknowledged_sequence;
    frame[9] = input.fault_code;
    frame[10] = 0;
    frame[11] = 0;
    const std::uint16_t crc = modbus_crc16(&frame[2], 10);
    frame[12] = static_cast<std::uint8_t>(crc & 0xffu);
    frame[13] = static_cast<std::uint8_t>((crc >> 8) & 0xffu);
    frame[14] = kFrameTail;
    return frame;
}

bool decode_fused_pose_frame(const std::uint8_t *data, std::size_t size,
                             FusedPoseFrame &output)
{
    if (!data || size != kFrameSize || data[0] != kFrameHead1 ||
        data[1] != kFrameHead2 || data[2] != kFusedPoseMessageType ||
        data[14] != kFrameTail) {
        return false;
    }
    const std::uint16_t expected = modbus_crc16(&data[2], 10);
    const std::uint16_t received = static_cast<std::uint16_t>(data[12]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[13]) << 8);
    if (expected != received) return false;
    output.sequence = data[3];
    output.x_mm = get_i16_be(&data[4]);
    output.y_mm = get_i16_be(&data[6]);
    output.heading_cdeg = get_u16_be(&data[8]);
    output.status = data[10];
    output.confidence_and_sigma = data[11];
    return true;
}

bool validate_relay_frame(const std::uint8_t *data, std::size_t size)
{
    if (!data || size != kFrameSize || data[0] != kFrameHead1 ||
        data[1] != kFrameHead2 || data[14] != kFrameTail) {
        return false;
    }
    if (data[2] != 0x11u && data[2] != 0x12u &&
        data[2] != kMissionCommandMessageType) {
        return false;
    }
    const std::uint16_t expected = modbus_crc16(&data[2], 10);
    const std::uint16_t received = static_cast<std::uint16_t>(data[12]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[13]) << 8);
    return expected == received;
}

F407FrameParser::F407FrameParser(Callback callback, StatusCallback status_callback)
    : callback_(std::move(callback)), status_callback_(std::move(status_callback))
{
}

void F407FrameParser::feed(const std::uint8_t *data, std::size_t size)
{
    if (!data) {
        return;
    }
    stats_.bytes += size;
    for (std::size_t i = 0; i < size; ++i) {
        consume(data[i]);
    }
}

void F407FrameParser::consume(std::uint8_t value)
{
    if (index_ == 0) {
        if (value == kFrameHead1) {
            frame_[index_++] = value;
        }
        return;
    }
    if (index_ == 1) {
        if (value == kFrameHead2) {
            frame_[index_++] = value;
        } else if (value != kFrameHead1) {
            index_ = 0;
        }
        return;
    }

    frame_[index_++] = value;
    if (index_ == kFrameSize) {
        validate_frame();
    }
}

void F407FrameParser::validate_frame()
{
    const std::uint16_t expected = modbus_crc16(&frame_[2], 10);
    const std::uint16_t received = static_cast<std::uint16_t>(frame_[12]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame_[13]) << 8);
    if (frame_[14] != kFrameTail ||
        (frame_[2] != kOdomMessageType && frame_[2] != kStmStatusMessageType)) {
        ++stats_.malformed;
        resynchronize();
        return;
    }
    if (received != expected) {
        ++stats_.crc_errors;
        resynchronize();
        return;
    }

    if (frame_[2] == kStmStatusMessageType) {
        StmStatusFrame status;
        status.sequence = frame_[3];
        status.flags = frame_[4];
        status.mode = frame_[5];
        status.camera_pitch_cdeg = get_u16_be(&frame_[6]);
        status.acknowledged_sequence = frame_[8];
        status.fault_code = frame_[9];
        if (have_status_sequence_) {
            const std::uint8_t difference =
                static_cast<std::uint8_t>(status.sequence - last_status_sequence_);
            if (difference == 0) {
                ++stats_.duplicates;
                index_ = 0;
                return;
            }
            if (difference > 1) {
                stats_.sequence_gaps += static_cast<std::uint8_t>(difference - 1);
            }
        }
        have_status_sequence_ = true;
        last_status_sequence_ = status.sequence;
        ++stats_.frames_ok;
        ++stats_.status_frames;
        if (status_callback_) status_callback_(status);
        index_ = 0;
        return;
    }

    EncoderFrame parsed;
    parsed.sequence = frame_[3];
    parsed.position[0] = get_u16_be(&frame_[4]);
    parsed.position[1] = get_u16_be(&frame_[6]);
    parsed.position[2] = get_u16_be(&frame_[8]);
    parsed.sample_period_ms = frame_[10];
    parsed.status = frame_[11];

    if (have_sequence_) {
        const std::uint8_t difference = static_cast<std::uint8_t>(parsed.sequence - last_sequence_);
        if (difference == 0) {
            ++stats_.duplicates;
            index_ = 0;
            return;
        } else if (difference > 1) {
            stats_.sequence_gaps += static_cast<std::uint8_t>(difference - 1);
        }
    }
    have_sequence_ = true;
    last_sequence_ = parsed.sequence;
    ++stats_.frames_ok;
    if (callback_) {
        callback_(parsed);
    }
    index_ = 0;
}

void F407FrameParser::resynchronize()
{
    std::size_t start = kFrameSize;
    for (std::size_t i = 1; i + 1 < kFrameSize; ++i) {
        if (frame_[i] == kFrameHead1 && frame_[i + 1] == kFrameHead2) {
            start = i;
        }
    }
    if (start < kFrameSize) {
        index_ = kFrameSize - start;
        std::copy(frame_.begin() + static_cast<std::ptrdiff_t>(start),
                  frame_.end(), frame_.begin());
    } else if (frame_[kFrameSize - 1] == kFrameHead1) {
        frame_[0] = kFrameHead1;
        index_ = 1;
    } else {
        index_ = 0;
    }
}

}  // namespace omni
