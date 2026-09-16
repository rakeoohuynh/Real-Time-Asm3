/* =====================================================================
 * central_command_server.h -- Central_Command_Server_Task.
 *
 * Receives OVERRIDE_COMMAND / MODE_SWITCH from the CC on the LC's
 * named channel with a blocking MsgReceive(), runs the LC-side safety
 * check, and replies. An accepted command is parked and applied by
 * Phase_Controller_Task at the next safe point, where the safety
 * condition is checked a SECOND time -- railway protection can begin
 * in the gap between acceptance and application, and it outranks any
 * CC command.
 * ===================================================================== */
#ifndef CENTRAL_COMMAND_SERVER_H
#define CENTRAL_COMMAND_SERVER_H

#include "lc_context.h"

void *central_command_server_task(void *arg);

/* Phase_Controller_Task: apply whatever the server accepted. */
void  apply_pending_cc_commands(lc_context_t *ctx);

#endif /* CENTRAL_COMMAND_SERVER_H */
