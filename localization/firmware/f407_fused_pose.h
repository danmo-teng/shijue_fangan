#ifndef F407_FUSED_POSE_H
#define F407_FUSED_POSE_H

#include <stdbool.h>
#include <stdint.h>

#define F407_FUSED_POSE_MESSAGE_TYPE       0x16U
#define F407_POSE_VALID                    (1U << 0)
#define F407_POSE_T265_GOOD                (1U << 1)
#define F407_POSE_WHEEL_ACTIVE             (1U << 2)
#define F407_POSE_OBSTACLE_GATE            (1U << 3)
#define F407_POSE_ODOM_FRESH               (1U << 4)
#define F407_POSE_INSIDE_FIELD             (1U << 5)
#define F407_POSE_T265_UPDATE_REJECTED     (1U << 6)

typedef struct {
  int16_t x_mm;
  int16_t y_mm;
  uint16_t heading_cdeg;
  uint32_t tick_ms;
  uint8_t sequence;
  uint8_t status;
  uint8_t tracker_confidence;
  uint8_t mapper_confidence;
  uint8_t position_sigma_cm;
  bool received;
} F407FusedPose;

/* Call after the common 15-byte parser has verified TYPE, tail and CRC. */
bool F407_FusedPoseDecodePayload(const uint8_t payload[8],
                                 uint8_t sequence,
                                 uint32_t tick_ms,
                                 F407FusedPose *pose);

bool F407_FusedPoseIsFresh(const F407FusedPose *pose,
                           uint32_t now_ms,
                           uint32_t timeout_ms);

#endif
