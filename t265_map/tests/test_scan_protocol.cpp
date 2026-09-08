#include "scan_protocol.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace t265_map::protocol;

int main()
{
    auto check = [](bool condition, const char *message) {
        if (!condition) std::cerr << "scan protocol test failed: " << message << '\n';
        return condition;
    };
    MotionCommandPayload command;
    command.command = kMotionMoveBody;
    command.flags = kFlagValid | kFlagKeepHeading | kFlagAckRequired;
    command.arg1 = 1000;
    command.arg2 = -250;
    command.speed = 300;
    const auto frame = build_motion_command(0x42, command);

    if (!check(frame[0] == kFrameHead1 && frame[1] == kFrameHead2, "frame header")) return 1;
    if (!check(frame[2] == kMsgScanMotionCommand && frame[3] == 0x42, "command type/sequence")) return 1;
    if (!check(frame[6] == 0x03 && frame[7] == 0xE8, "forward argument")) return 1;
    if (!check(frame[8] == 0xFF && frame[9] == 0x06, "left argument")) return 1;

    const std::vector<std::size_t> chunks{1, 2, 4, 3, 5};
    std::vector<std::uint8_t> received_types;
    std::vector<std::uint8_t> received_sequences;
    StreamParser parser([&](std::uint8_t type, std::uint8_t sequence,
                            const std::array<std::uint8_t, kPayloadSize> &) {
        received_types.push_back(type);
        received_sequences.push_back(sequence);
    });
    std::size_t offset = 0;
    for (const std::size_t chunk : chunks) {
        const std::size_t end = offset + chunk < frame.size()
            ? offset + chunk : frame.size();
        parser.feed(frame.data() + offset, end - offset);
        offset = end;
        if (offset == frame.size()) break;
    }
    if (!check(received_types.size() == 1, "split frame callback count")) return 1;
    if (!check(received_types[0] == kMsgScanMotionCommand, "split frame type")) return 1;
    if (!check(received_sequences[0] == 0x42, "split frame sequence")) return 1;

    std::uint8_t type = 0;
    std::uint8_t sequence = 0;
    std::array<std::uint8_t, kPayloadSize> payload{};
    if (!check(parse_frame(frame.data(), frame.size(), type, sequence, payload), "command CRC")) return 1;
    if (!check(type == kMsgScanMotionCommand && sequence == 0x42, "command parse")) return 1;

    MotionCommandPayload full_turn = command;
    full_turn.command = kMotionTurnRelative;
    full_turn.arg1 = 3600;  // 360.0 degrees in the signed 0.1-degree field.
    const auto full_turn_frame = build_motion_command(0x43, full_turn);
    if (!check(full_turn_frame[6] == 0x0E && full_turn_frame[7] == 0x10,
               "360 degree argument")) return 1;

    std::array<std::uint8_t, kFrameSize> bad = frame;
    bad[2] = kMsgOdom;
    bad[4] = 0x12;
    bad[5] = 0x34;
    bad[6] = 0x56;
    bad[7] = 0x78;
    bad[8] = 0x9A;
    bad[9] = 0xBC;
    bad[10] = 10;
    bad[11] = 0x07;
    const auto crc = crc16_modbus(bad.data() + 2, 10);
    bad[12] = static_cast<std::uint8_t>(crc & 0xFFu);
    bad[13] = static_cast<std::uint8_t>((crc >> 8) & 0xFFu);
    bad[14] = kFrameTail;
    if (!check(parse_frame(bad.data(), bad.size(), type, sequence, payload), "odom CRC")) return 1;
    OdomPayload odom;
    if (!check(decode_odom(sequence, payload, odom), "odom decode")) return 1;
    if (!check(odom.m1_count == 0x1234 && odom.m2_count == 0x5678, "odom counts")) return 1;
    if (!check(odom.dt_ms == 10, "odom period")) return 1;

    bad[12] ^= 0x01;
    StreamParser crc_parser;
    crc_parser.feed(bad.data(), bad.size());
    if (!check(crc_parser.frames() == 0, "bad CRC frame rejected")) return 1;
    if (!check(crc_parser.crc_errors() == 1, "bad CRC counted")) return 1;
    return 0;
}
