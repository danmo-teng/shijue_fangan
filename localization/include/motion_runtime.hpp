#pragma once

// Native motion owns only duplicated, already-bound local input sockets. It
// never opens the chassis UART or the T265 SDK. Returned strings use malloc;
// release each with omni_motion_runtime_free_string, including error replies.
extern "C" {
void *omni_motion_runtime_create(const char *config_json);
char *omni_motion_runtime_command(void *handle, const char *command_json);
char *omni_motion_runtime_snapshot(void *handle, const char *query_json);
void omni_motion_runtime_close(void *handle);
// Join and finish diagnostic writers, return final snapshot, then destroy.
char *omni_motion_runtime_finish(void *handle);
void omni_motion_runtime_free_string(char *value);
const char *omni_motion_runtime_last_error();
}
