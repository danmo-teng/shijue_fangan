#include "f407_odom_protocol.h"

static void put_u16_be(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)(value >> 8);
  destination[1] = (uint8_t)(value & 0xFFU);
}
uint16_t F407_OdomCrc16(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < size; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = ((crc & 1U) != 0U) ?
          (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

void F407_OdomBuildFrame(const F407OdomPayload *payload,
                         uint8_t frame[F407_ODOM_FRAME_SIZE])
{
  if ((payload == 0) || (frame == 0)) {
    return;
  }
  frame[0] = 0xA3U;
  frame[1] = 0xB3U;
  frame[2] = F407_ODOM_MESSAGE_TYPE;
  frame[3] = payload->sequence;
  put_u16_be(&frame[4], payload->position[0]);
  put_u16_be(&frame[6], payload->position[1]);
  put_u16_be(&frame[8], payload->position[2]);
  frame[10] = payload->sample_period_ms;
  frame[11] = payload->status;
  const uint16_t crc = F407_OdomCrc16(&frame[2], 10U);
  frame[12] = (uint8_t)(crc & 0xFFU);
  frame[13] = (uint8_t)(crc >> 8);
  frame[14] = 0xC3U;
}
