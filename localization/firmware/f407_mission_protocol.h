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

#define F407_GRAB_MIN_ACTION_MS 2000U
#define F407_NAV_NORMAL_MAX_AGE_MS 250U
#define F407_NAV_SLOW_MAX_AGE_MS 1000U
#define F407_NAV_SLOW_SPEED_MMPS 250U

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

typedef enum {
  F407_MISSION_ACTION_STOP_WAIT = 0,
  F407_MISSION_ACTION_START_GRAB,
  F407_MISSION_ACTION_WAIT_GRIPPER,
  F407_MISSION_ACTION_NAV_NORMAL,
  F407_MISSION_ACTION_NAV_SLOW,
  F407_MISSION_ACTION_ABORTED
} F407MissionAction;

typedef struct {
  bool grab_in_progress;
  bool gripper_closed;
  bool navigation_active;
  bool temporary_stop;
  bool aborted;
  uint32_t grab_started_ms;
  uint32_t last_navigation_command_ms;
  uint16_t navigation_heading_cdeg;
  uint8_t acknowledged_sequence;
} F407MissionRuntime;

void F407_StmStatusBuildFrame(const F407StmStatusPayload *payload,
                              uint8_t frame[F407_MISSION_FRAME_SIZE]);

bool F407_MissionDecodePayload(const uint8_t payload[8], uint8_t sequence,
                               F407MissionCommand *result);

/* Repeated GRAB_CONFIRMED frames are a reliability heartbeat.  Start the
 * actuator only once; still acknowledge every valid frame sequence. */
bool F407_MissionShouldStartGrab(const F407MissionCommand *command,
                                 bool grab_in_progress,
                                 bool gripper_closed);

void F407_MissionRuntimeInit(F407MissionRuntime *runtime);

/* Apply one already decoded/validated command. Every valid sequence is
 * acknowledged, including idempotent repeated GRAB commands. */
F407MissionAction F407_MissionApplyCommand(F407MissionRuntime *runtime,
                                           const F407MissionCommand *command,
                                           uint32_t now_ms);

/* Set GRIPPER_CLOSED only when both physical actuators report completion and
 * the two-second minimum motion window has elapsed. */
bool F407_MissionUpdateGripper(F407MissionRuntime *runtime,
                               bool left_claw_done,
                               bool right_claw_done,
                               uint32_t now_ms);

/* Call before building each 50ms TYPE=0x17 frame. The caller owns sequence,
 * mode, camera pitch and fault fields. */
void F407_MissionFillStatus(const F407MissionRuntime *runtime,
                            F407StmStatusPayload *status);

/* Navigation timeout is recoverable: <=250ms normal, <=1000ms slow, then
 * stopped while navigation_active remains latched. Invalid pose stops now;
 * a valid pose plus a fresh NAV command resumes. ABORT never resumes. */
F407MissionAction F407_MissionNavigationPolicy(
    const F407MissionRuntime *runtime,
    uint32_t now_ms,
    bool fused_pose_valid);

uint16_t F407_MissionSpeedLimitMmps(F407MissionAction action,
                                    uint16_t normal_speed_mmps);

#endif
