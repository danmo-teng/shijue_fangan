#include "f407_mission_protocol.h"

#include <assert.h>
#include <stdio.h>

int main(void) {
  const uint8_t grab_payload[8] = {
      F407_CMD_GRAB_CONFIRMED, F407_CMD_VALID, 0, 0, 0, 0, 0, 0};
  F407MissionCommand command;
  assert(F407_MissionDecodePayload(grab_payload, 7U, &command));
  assert(F407_MissionShouldStartGrab(&command, false, false));
  assert(!F407_MissionShouldStartGrab(&command, true, false));
  assert(!F407_MissionShouldStartGrab(&command, false, true));

  command.command = F407_CMD_NAVIGATE_WAYPOINT;
  assert(!F407_MissionShouldStartGrab(&command, false, false));
  assert(!F407_MissionShouldStartGrab(NULL, false, false));

  F407MissionRuntime runtime;
  F407_MissionRuntimeInit(&runtime);
  command.command = F407_CMD_GRAB_CONFIRMED;
  command.sequence = 10U;
  assert(F407_MissionApplyCommand(&runtime, &command, 1000U) ==
         F407_MISSION_ACTION_START_GRAB);
  assert(runtime.grab_in_progress && runtime.acknowledged_sequence == 10U);

  /* Repeated commands acknowledge their new sequence without restarting the
   * physical two-second motion window. */
  command.sequence = 11U;
  assert(F407_MissionApplyCommand(&runtime, &command, 1500U) ==
         F407_MISSION_ACTION_WAIT_GRIPPER);
  assert(runtime.grab_started_ms == 1000U && runtime.acknowledged_sequence == 11U);
  assert(!F407_MissionUpdateGripper(&runtime, true, true, 2999U));
  assert(!F407_MissionUpdateGripper(&runtime, true, false, 3000U));
  assert(F407_MissionUpdateGripper(&runtime, true, true, 3000U));
  assert(runtime.gripper_closed && !runtime.grab_in_progress);

  F407StmStatusPayload status = {0};
  F407_MissionFillStatus(&runtime, &status);
  assert((status.flags & F407_STM_GRIPPER_CLOSED) != 0U);
  assert(status.acknowledged_sequence == 11U);

  command.command = F407_CMD_NAVIGATE_WAYPOINT;
  command.flags = F407_CMD_VALID | F407_CMD_DRIVE_STRAIGHT |
                  F407_CMD_USE_FINAL_HEADING;
  command.heading_cdeg = 23362U;
  command.sequence = 12U;
  assert(F407_MissionApplyCommand(&runtime, &command, 4000U) ==
         F407_MISSION_ACTION_NAV_NORMAL);
  assert(runtime.navigation_heading_cdeg == 23362U);
  assert(F407_MissionNavigationPolicy(&runtime, 4250U, true) ==
         F407_MISSION_ACTION_NAV_NORMAL);
  assert(F407_MissionNavigationPolicy(&runtime, 4251U, true) ==
         F407_MISSION_ACTION_NAV_SLOW);
  assert(F407_MissionSpeedLimitMmps(F407_MISSION_ACTION_NAV_SLOW, 600U) == 250U);
  assert(F407_MissionNavigationPolicy(&runtime, 5000U, true) ==
         F407_MISSION_ACTION_NAV_SLOW);
  assert(F407_MissionNavigationPolicy(&runtime, 5001U, true) ==
         F407_MISSION_ACTION_STOP_WAIT);
  assert(runtime.navigation_active);
  assert(F407_MissionSpeedLimitMmps(F407_MISSION_ACTION_STOP_WAIT, 600U) == 0U);

  /* A fresh NAV resumes after command timeout. Pose invalidity is also a
   * temporary stop and resumes when pose and direction are fresh again. */
  command.sequence = 13U;
  assert(F407_MissionApplyCommand(&runtime, &command, 5100U) ==
         F407_MISSION_ACTION_NAV_NORMAL);
  assert(F407_MissionNavigationPolicy(&runtime, 5101U, false) ==
         F407_MISSION_ACTION_STOP_WAIT);
  assert(F407_MissionNavigationPolicy(&runtime, 5102U, true) ==
         F407_MISSION_ACTION_NAV_NORMAL);

  F407MissionCommand stop = command;
  stop.command = F407_CMD_STOP;
  stop.sequence = 14U;
  assert(F407_MissionApplyCommand(&runtime, &stop, 5200U) ==
         F407_MISSION_ACTION_STOP_WAIT);
  assert(runtime.navigation_active && !runtime.aborted);
  command.sequence = 15U;
  assert(F407_MissionApplyCommand(&runtime, &command, 5300U) ==
         F407_MISSION_ACTION_NAV_NORMAL);

  F407MissionCommand abort = command;
  abort.command = F407_CMD_ABORT;
  abort.sequence = 16U;
  assert(F407_MissionApplyCommand(&runtime, &abort, 5400U) ==
         F407_MISSION_ACTION_ABORTED);
  command.sequence = 17U;
  assert(F407_MissionApplyCommand(&runtime, &command, 5500U) ==
         F407_MISSION_ACTION_ABORTED);
  assert(F407_MissionNavigationPolicy(&runtime, 5500U, true) ==
         F407_MISSION_ACTION_ABORTED);

  /* NAV received before physical closure never starts the gripper fallback. */
  F407MissionRuntime not_closed;
  F407_MissionRuntimeInit(&not_closed);
  assert(F407_MissionApplyCommand(&not_closed, &command, 6000U) ==
         F407_MISSION_ACTION_WAIT_GRIPPER);
  assert(!not_closed.grab_in_progress && !not_closed.navigation_active);

  puts("F407 mission idempotency PASS");
  return 0;
}
