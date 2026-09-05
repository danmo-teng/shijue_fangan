#include "f407_mission_protocol.h"

#include <stddef.h>
#include <string.h>

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
  if (result->command > F407_CMD_RETURN_CENTER || result->command == 1U) return false;
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

void F407_MissionRuntimeInit(F407MissionRuntime *runtime) {
  if (runtime == NULL) return;
  memset(runtime, 0, sizeof(*runtime));
  runtime->temporary_stop = true;
}

static bool is_navigation_command(uint8_t command) {
  return command == F407_CMD_NAVIGATE_WAYPOINT ||
         command == F407_CMD_ALIGN_SAFE_ZONE ||
         command == F407_CMD_ENTER_SAFE_ZONE;
}

F407MissionAction F407_MissionApplyCommand(F407MissionRuntime *runtime,
                                           const F407MissionCommand *command,
                                           uint32_t now_ms) {
  if (runtime == NULL || command == NULL) return F407_MISSION_ACTION_STOP_WAIT;
  runtime->acknowledged_sequence = command->sequence;
  if (runtime->aborted) return F407_MISSION_ACTION_ABORTED;

  if (command->command == F407_CMD_ABORT) {
    runtime->aborted = true;
    runtime->temporary_stop = true;
    runtime->navigation_active = false;
    return F407_MISSION_ACTION_ABORTED;
  }
  if (command->command == F407_CMD_STOP ||
      command->command == F407_CMD_TASK_COMPLETE) {
    runtime->temporary_stop = true;
    return F407_MISSION_ACTION_STOP_WAIT;
  }
  if (command->command == F407_CMD_GRAB_CONFIRMED) {
    runtime->temporary_stop = true;
    runtime->navigation_active = false;
    if (F407_MissionShouldStartGrab(command, runtime->grab_in_progress,
                                    runtime->gripper_closed)) {
      runtime->grab_in_progress = true;
      runtime->grab_started_ms = now_ms;
      return F407_MISSION_ACTION_START_GRAB;
    }
    return F407_MISSION_ACTION_WAIT_GRIPPER;
  }
  if (is_navigation_command(command->command)) {
    /* Do not retain the legacy "NAV starts grabbing" fallback. */
    if (!runtime->gripper_closed) {
      runtime->temporary_stop = true;
      return F407_MISSION_ACTION_WAIT_GRIPPER;
    }
    if ((command->flags & F407_CMD_USE_FINAL_HEADING) == 0U ||
        command->heading_cdeg >= 36000U) {
      runtime->temporary_stop = true;
      return F407_MISSION_ACTION_STOP_WAIT;
    }
    runtime->navigation_active = true;
    runtime->temporary_stop = false;
    runtime->last_navigation_command_ms = now_ms;
    runtime->navigation_heading_cdeg = command->heading_cdeg;
    return F407_MISSION_ACTION_NAV_NORMAL;
  }
  runtime->temporary_stop = true;
  return F407_MISSION_ACTION_STOP_WAIT;
}

bool F407_MissionUpdateGripper(F407MissionRuntime *runtime,
                               bool left_claw_done,
                               bool right_claw_done,
                               uint32_t now_ms) {
  if (runtime == NULL || !runtime->grab_in_progress ||
      !left_claw_done || !right_claw_done) {
    return false;
  }
  if ((uint32_t)(now_ms - runtime->grab_started_ms) < F407_GRAB_MIN_ACTION_MS) {
    return false;
  }
  runtime->grab_in_progress = false;
  runtime->gripper_closed = true;
  return true;
}

void F407_MissionFillStatus(const F407MissionRuntime *runtime,
                            F407StmStatusPayload *status) {
  if (runtime == NULL || status == NULL) return;
  status->acknowledged_sequence = runtime->acknowledged_sequence;
  if (runtime->gripper_closed) {
    status->flags |= F407_STM_GRIPPER_CLOSED;
  } else {
    status->flags &= (uint8_t)~F407_STM_GRIPPER_CLOSED;
  }
}

F407MissionAction F407_MissionNavigationPolicy(
    const F407MissionRuntime *runtime,
    uint32_t now_ms,
    bool fused_pose_valid) {
  if (runtime == NULL || runtime->aborted) return F407_MISSION_ACTION_ABORTED;
  if (!fused_pose_valid || runtime->temporary_stop ||
      !runtime->navigation_active || !runtime->gripper_closed) {
    return F407_MISSION_ACTION_STOP_WAIT;
  }
  const uint32_t age_ms = now_ms - runtime->last_navigation_command_ms;
  if (age_ms <= F407_NAV_NORMAL_MAX_AGE_MS) {
    return F407_MISSION_ACTION_NAV_NORMAL;
  }
  if (age_ms <= F407_NAV_SLOW_MAX_AGE_MS) {
    return F407_MISSION_ACTION_NAV_SLOW;
  }
  return F407_MISSION_ACTION_STOP_WAIT;
}

uint16_t F407_MissionSpeedLimitMmps(F407MissionAction action,
                                    uint16_t normal_speed_mmps) {
  if (action == F407_MISSION_ACTION_NAV_NORMAL) return normal_speed_mmps;
  if (action == F407_MISSION_ACTION_NAV_SLOW) {
    return normal_speed_mmps < F407_NAV_SLOW_SPEED_MMPS
               ? normal_speed_mmps
               : F407_NAV_SLOW_SPEED_MMPS;
  }
  return 0U;
}
