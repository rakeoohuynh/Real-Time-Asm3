/*
 * central_command_server.h -- Central_Command_Server_Task.
 *
 * Receives OVERRIDE_COMMAND and MODE_SWITCH from the CC, checks whether
 * an override is safe right now, and replies. Accepted commands are
 * applied later by Phase_Controller_Task, which checks safety again:
 * railway protection can start between acceptance and application, and
 * it always wins over a CC command.
 */
#ifndef CENTRAL_COMMAND_SERVER_H
#define CENTRAL_COMMAND_SERVER_H

#include "lc_context.h"

void *central_command_server_task(void *arg);

/* Phase_Controller_Task: applies whatever the server accepted. */
void  apply_pending_cc_commands(lc_context_t *ctx);

#endif /* CENTRAL_COMMAND_SERVER_H */
