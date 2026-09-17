/* =============================================================
 * cc_main.c - Central Controller (status display)
 *
 * Assessment 2 design principle (Section 7): "The Central Controller
 * (CC) monitors, logs, schedules, and issues high-level commands; it
 * never directly controls physical signal outputs." For this phase
 * (fixed_time only, no MODE_SWITCH/OVERRIDE_COMMAND yet), the CC's
 * role is reduced to what UC-01 and UC-06 actually require: receiving
 * STATUS_UPDATE from each LC and displaying it, satisfying the current
 * assessment's requirement for "a central control room... which
 * monitors the traffic lights and displays the status".
 *
 * name_attach() with a LOCAL attach point (flags=0) registers this
 * process under /dev/name/local/<name>. Qnet still exposes the entire
 * remote namespace under /net/<node>/..., so a Local Controller on a
 * different node reaches it at /net/<this_node>/dev/name/local/<name> -
 * see Lab 6 Topic 2/3 (named connections, Qnet).
 *
 * NAME_FLAG_ATTACH_GLOBAL (registering under /dev/name/global/<name>
 * instead) was deliberately NOT used: it requires the Global Name
 * Service (gns) resource manager to be running on this node, which a
 * minimal mkqnximage build does not start by default. The lab's own
 * Task 1A server code notes exactly this trade-off, defaulting to the
 * local attach point for the same reason.
 * ============================================================= */
#include "protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#define ATTACH_POINT "traffic_cc_status"

static volatile sig_atomic_t g_shutdown = 0;

static void handle_shutdown_signal(int sig)
{
    (void)sig;
    g_shutdown = 1; /* async-signal-safe; interrupts MsgReceive with EINTR */
}

static void print_timestamp(void)
{
    time_t     now = time(NULL);
    struct tm *t   = localtime(&now);
    printf("%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
}

int main(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_shutdown_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    name_attach_t *attach = name_attach(NULL, ATTACH_POINT, 0);
    if (attach == NULL) {
        fprintf(stderr, "ERROR: name_attach('%s') failed: %s\n", ATTACH_POINT, strerror(errno));
        fprintf(stderr, "Is another Central Controller already running?\n");
        return EXIT_FAILURE;
    }

    printf("=========================================================\n");
    printf(" QNX Central Controller - status display\n");
    printf(" Listening on (local): %s\n", ATTACH_POINT);
    printf(" Reachable from another node at: /net/<this-node>/dev/name/local/%s\n", ATTACH_POINT);
    printf("=========================================================\n");

    union { struct _pulse pulse; } msg;
    int rcvid;

    for (;;) {
        rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) {
                if (g_shutdown) break;
                continue;
            }
            perror("ERROR: MsgReceive");
            break;
        }
        if (rcvid != 0) continue; /* CC only ever receives pulses in this phase */

        if (msg.pulse.code == PULSE_STATUS_UPDATE) {
            int                value = msg.pulse.value.sival_int;
            intersection_id_t  iid   = (intersection_id_t)STATUS_UNPACK_INTERSECTION(value);
            phase_state_t       ph    = (phase_state_t)STATUS_UNPACK_PHASE(value);

            print_timestamp();
            printf("  STATUS_UPDATE  intersection=%-4s phase=%-10s mode=FIXED_TIMING\n",
                   intersection_name(iid), phase_name(ph));
            fflush(stdout);
        }
        /* Other pulse codes (e.g. _PULSE_CODE_DISCONNECT on client
         * teardown) are expected and intentionally ignored. */
    }

    name_detach(attach, 0);
    printf("Central Controller terminated cleanly.\n");
    return EXIT_SUCCESS;
}
