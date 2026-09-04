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
  puts("F407 mission idempotency PASS");
  return 0;
}
