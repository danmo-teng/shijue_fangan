#ifndef F407_MISSION_PROTOCOL_H
#define F407_MISSION_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define F407_MISSION_FRAME_SIZE 15U
#define F407_STM_STATUS_TYPE 0x17U
#define F407_MISSION_COMMAND_TYPE 0x18U

#define F407_STM_CLAW_VISIBLE   (1U << 0)
#define F407_STM_GRIPPER_CLOSED (1U << 1)
#define F407_STM_MOTORS_ACTIVE  (1U << 2)
#define F407_STM_AUTO_APPROACH  (1U << 3)
#define F407_STM_FAULT          (1U << 7)

#define F407_CMD_VALID             (1U << 0)
#define F407_CMD_DRIVE_STRAIGHT    (1U << 1)
#define F407_CMD_USE_FINAL_HEADING (1U << 2)
#define F407_CMD_RED_SIDE          (1U << 3)

typedef enum {
  F407_CMD_STOP = 0,
  F407_CMD_GRAB_CONFIRMED = 2,
  F407_CMD_NAVIGATE_WAYPOINT = 3,
  F407_CMD_ALIGN_SAFE_ZONE = 4,
  F407_CMD_ENTER_SAFE_ZONE = 5,
  F407_CMD_TASK_COMPLETE = 6,
  F407_CMD_ABORT = 7
} F407MissionCommandCode;

typedef struct {
  uint8_t sequence;
  uint8_t flags;
  uint8_t mode;
  uint16_t camera_pitch_cdeg;
  uint8_t acknowledged_sequence;
  uint8_t fault_code;
} F407StmStatusPayload;

typedef struct {
  uint8_t sequence;
  uint8_t command;
  uint8_t flags;
  int16_t target_x_mm;
  int16_t target_y_mm;
  uint16_t heading_cdeg;
} F407MissionCommand;

void F407_StmStatusBuildFrame(const F407StmStatusPayload *payload,
                              uint8_t frame[F407_MISSION_FRAME_SIZE]);

bool F407_MissionDecodePayload(const uint8_t payload[8], uint8_t sequence,
                               F407MissionCommand *result);

/* Repeated GRAB_CONFIRMED frames are a reliability heartbeat.  Start the
 * actuator only once; still acknowledge every valid frame sequence. */
bool F407_MissionShouldStartGrab(const F407MissionCommand *command,
                                 bool grab_in_progress,
                                 bool gripper_closed);

#endif
