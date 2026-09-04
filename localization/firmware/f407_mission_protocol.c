#include "f407_mission_protocol.h"

#include <stddef.h>

static uint16_t crc16_modbus(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFFU;
  size_t index;
  uint8_t bit;
  for (index = 0; index < size; ++index) {
    crc ^= data[index];
    for (bit = 0; bit < 8U; ++bit) {
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U)
                       : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

static void put_u16_be(uint8_t *data, uint16_t value) {
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

static uint16_t get_u16_be(const uint8_t *data) {
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

void F407_StmStatusBuildFrame(const F407StmStatusPayload *payload,
                              uint8_t frame[F407_MISSION_FRAME_SIZE]) {
  uint16_t crc;
  frame[0] = 0xA3U;
  frame[1] = 0xB3U;
  frame[2] = F407_STM_STATUS_TYPE;
  frame[3] = payload->sequence;
  frame[4] = payload->flags;
  frame[5] = payload->mode;
  put_u16_be(&frame[6], payload->camera_pitch_cdeg);
  frame[8] = payload->acknowledged_sequence;
  frame[9] = payload->fault_code;
  frame[10] = 0U;
  frame[11] = 0U;
  crc = crc16_modbus(&frame[2], 10U);
  frame[12] = (uint8_t)crc;
  frame[13] = (uint8_t)(crc >> 8);
  frame[14] = 0xC3U;
}

bool F407_MissionDecodePayload(const uint8_t payload[8], uint8_t sequence,
                               F407MissionCommand *result) {
  if (payload == NULL || result == NULL) return false;
  result->sequence = sequence;
  result->command = payload[0];
  result->flags = payload[1];
  result->target_x_mm = (int16_t)get_u16_be(&payload[2]);
  result->target_y_mm = (int16_t)get_u16_be(&payload[4]);
  result->heading_cdeg = get_u16_be(&payload[6]);
  if ((result->flags & F407_CMD_VALID) == 0U) return false;
  if (result->command > F407_CMD_ABORT || result->command == 1U) return false;
  return result->heading_cdeg < 36000U;
}

bool F407_MissionShouldStartGrab(const F407MissionCommand *command,
                                 bool grab_in_progress,
                                 bool gripper_closed) {
  if (command == NULL || command->command != F407_CMD_GRAB_CONFIRMED) {
    return false;
  }
  return !grab_in_progress && !gripper_closed;
}
