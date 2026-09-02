#include "f407_fused_pose.h"

static uint16_t get_u16_be(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

bool F407_FusedPoseDecodePayload(const uint8_t payload[8],
                                 uint8_t sequence,
                                 uint32_t tick_ms,
                                 F407FusedPose *pose)
{
  if ((payload == 0) || (pose == 0)) {
    return false;
  }
  if (pose->received && (pose->sequence == sequence)) {
    return false;
  }
  const uint16_t heading_cdeg = get_u16_be(&payload[4]);
  if ((heading_cdeg >= 36000U) || ((payload[6] & 0x80U) != 0U)) {
    return false;
  }
  pose->x_mm = (int16_t)get_u16_be(&payload[0]);
  pose->y_mm = (int16_t)get_u16_be(&payload[2]);
  pose->heading_cdeg = heading_cdeg;
  pose->status = payload[6];
  pose->tracker_confidence = payload[7] & 0x03U;
  pose->mapper_confidence = (payload[7] >> 2) & 0x03U;
  pose->position_sigma_cm = (payload[7] >> 4) & 0x0FU;
  pose->sequence = sequence;
  pose->tick_ms = tick_ms;
  pose->received = true;
  return true;
}

bool F407_FusedPoseIsFresh(const F407FusedPose *pose,
                           uint32_t now_ms,
                           uint32_t timeout_ms)
{
  return (pose != 0) && pose->received &&
         ((pose->status & F407_POSE_VALID) != 0U) &&
         ((uint32_t)(now_ms - pose->tick_ms) <= timeout_ms);
}
