#ifndef F407_ODOM_PROTOCOL_H
#define F407_ODOM_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define F407_ODOM_FRAME_SIZE       15U
#define F407_ODOM_MESSAGE_TYPE     0x15U
#define F407_ODOM_M1_VALID         (1U << 0)
#define F407_ODOM_M2_VALID         (1U << 1)
#define F407_ODOM_M3_VALID         (1U << 2)
#define F407_ODOM_COUNTER_RESET    (1U << 3)
#define F407_ODOM_ENCODER_FAULT    (1U << 4)

typedef struct {
  uint16_t position[3];
  uint8_t sample_period_ms;
  uint8_t status;
  uint8_t sequence;
} F407OdomPayload;

uint16_t F407_OdomCrc16(const uint8_t *data, size_t size);
void F407_OdomBuildFrame(const F407OdomPayload *payload,
                         uint8_t frame[F407_ODOM_FRAME_SIZE]);

#endif
