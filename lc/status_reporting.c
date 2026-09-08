/* =============================================================
 * status_reporting.c - Status_Reporting_Task
 * See status_reporting.h for design notes.
 * ============================================================= */
#include "status_reporting.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/dispatch.h> /* name_open() / name_close() */

/* Throttle reconnect attempts while the CC link is down, so a
 * permanently-offline CC does not cause a name_open() retry storm. */
#define RECONNECT_EVERY_N_EVENTS 10

void *status_reporting_task(void *arg)
{
    status_reporting_args_t *args   = (status_reporting_args_t *)arg;
    intersection_id_t         id     = args->intersection_id;
    int                        chid   = args->chid;
    const char                *cc_name = args->cc_name;

    int cc_coid          = name_open(cc_name, 0);
    int central_link_up  = (cc_coid != -1);
    int events_since_try  = 0;

    if (central_link_up) {
        printf("[%s] Status_Reporting_Task connected to Central Controller ('%s').\n",
               intersection_name(id), cc_name);
    } else {
        printf("[%s] Status_Reporting_Task: Central Controller unavailable at startup - "
               "operating autonomously (A34), will retry in background.\n",
               intersection_name(id));
    }

    union { struct _pulse pulse; } msg;
    int rcvid;

    for (;;) {
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            perror("ERROR: Status_Reporting_Task MsgReceive");
            break;
        }
        if (rcvid != 0) continue; /* this channel only ever carries pulses */

        if (msg.pulse.code == PULSE_SHUTDOWN) {
            printf("[%s] Status_Reporting_Task shutting down.\n", intersection_name(id));
            break;
        }
        if (msg.pulse.code != PULSE_STATUS_EVENT) continue;

        if (!central_link_up && (++events_since_try >= RECONNECT_EVERY_N_EVENTS)) {
            events_since_try = 0;
            cc_coid = name_open(cc_name, 0);
            if (cc_coid != -1) {
                central_link_up = 1;
                printf("[%s] Status_Reporting_Task: Central Controller link RESTORED.\n",
                       intersection_name(id));
            }
        }

        if (central_link_up) {
            if (MsgSendPulse(cc_coid, -1, PULSE_STATUS_UPDATE, msg.pulse.value.sival_int) == -1) {
                fprintf(stderr, "[%s] WARNING: Central Controller link LOST (%s) - "
                        "continuing autonomously.\n", intersection_name(id), strerror(errno));
                name_close(cc_coid);
                cc_coid          = -1;
                central_link_up  = 0;
                events_since_try = 0;
            }
        }
        /* central_link_up == 0 and not yet due for a retry: drop this
         * one status event locally. LC operation is unaffected - only
         * CC visibility is delayed until the link is restored. */
    }

    if (cc_coid != -1) name_close(cc_coid);
    return NULL;
}
