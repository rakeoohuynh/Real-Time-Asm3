/* =============================================================
 * lc_main.c - Local Controller process entry point
 *
 * One LC executable is run once per intersection (argv[1] = "I1" or
 * "I2"), matching Assessment 2 Section 7: "I1 is fully expanded to
 * show the complete LC task architecture. I2-I6 use the same
 * architecture." Each run is an independent QNX process, intended to
 * run on its own QNX node (see README.md for the vm1/vm2 mapping).
 *
 * Process layout (Assessment 2 Fig. 18, reduced to what UC-01
 * fixed-timing operation needs for this phase):
 *
 *   main thread      -> Phase_Controller_Task  (safety-critical core FSM)
 *   pthread           -> Signal_Output_Task     (EVENT_DRIVEN)
 *   pthread           -> Status_Reporting_Task  (EVENT_DRIVEN + BACKGROUND)
 *
 * Pedestrian_Input_Task, Vehicle_Sensor_Task and the railway-crossing
 * subsystem are NOT part of this phase (fixed_time only, I1/I2 only)
 * and are intentionally omitted - see README.md "Scope" section.
 * ============================================================= */
#include "protocol.h"
#include "phase_controller.h"
#include "signal_output.h"
#include "status_reporting.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>

#define DEFAULT_CC_NAME "traffic_cc_status"

static volatile sig_atomic_t g_shutdown = 0;

static void handle_shutdown_signal(int sig)
{
    (void)sig;
    /* Async-signal-safe only: no IPC calls here. This interrupts the
     * blocking MsgReceive() in phase_controller_run() with EINTR; the
     * main loop checks g_shutdown and exits cleanly from there. */
    g_shutdown = 1;
}

static void print_usage(const char *prog)
{
    fprintf(stderr, "Usage: %s <I1|I2> [central_controller_name]\n", prog);
    fprintf(stderr, "  central_controller_name defaults to \"%s\" (same QNX node).\n",
            DEFAULT_CC_NAME);
    fprintf(stderr, "  For a Central Controller on a different QNX node, pass a QNET path,\n");
    fprintf(stderr, "  e.g. /net/vm1/dev/name/local/%s\n", DEFAULT_CC_NAME);
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    intersection_id_t id;
    if (strcmp(argv[1], "I1") == 0) {
        id = INTERSECTION_I1;
    } else if (strcmp(argv[1], "I2") == 0) {
        id = INTERSECTION_I2;
    } else {
        fprintf(stderr, "ERROR: unknown intersection '%s' (this phase implements I1 and I2 only)\n",
                argv[1]);
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const char *cc_name = (argc >= 3) ? argv[2] : DEFAULT_CC_NAME;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_shutdown_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    printf("=========================================================\n");
    printf(" QNX Traffic Light Local Controller - %s (fixed_time mode)\n", intersection_name(id));
    printf(" Central Controller attach point: %s\n", cc_name);
    printf("=========================================================\n");

    /* Channels are created here (main thread) before the owning
     * threads start, so both chids are valid and race-free the moment
     * Phase_Controller_Task tries to connect to them. */
    int chid_signal = ChannelCreate(0);
    if (chid_signal == -1) {
        perror("ERROR: ChannelCreate(Signal_Output_Task)");
        return EXIT_FAILURE;
    }

    int chid_status = ChannelCreate(0);
    if (chid_status == -1) {
        perror("ERROR: ChannelCreate(Status_Reporting_Task)");
        return EXIT_FAILURE;
    }

    signal_output_args_t sig_args = { id, chid_signal };
    pthread_t             sig_thread;
    if (pthread_create(&sig_thread, NULL, signal_output_task, &sig_args) != 0) {
        fprintf(stderr, "ERROR: Failed to create Signal_Output_Task thread: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }

    status_reporting_args_t stat_args = { id, chid_status, cc_name };
    pthread_t                stat_thread;
    if (pthread_create(&stat_thread, NULL, status_reporting_task, &stat_args) != 0) {
        fprintf(stderr, "ERROR: Failed to create Status_Reporting_Task thread: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }

    /* Phase_Controller_Task runs on the main thread and blocks here
     * until shutdown - it is the safety-critical core of the LC. */
    phase_controller_run(id, chid_signal, chid_status, &g_shutdown);

    /* Clean shutdown (Test 7): wake the other two tasks so they can
     * exit their MsgReceive loops, then join them before releasing
     * shared resources. */
    int coid;

    coid = ConnectAttach(ND_LOCAL_NODE, getpid(), chid_signal, _NTO_SIDE_CHANNEL, 0);
    if (coid != -1) {
        MsgSendPulse(coid, -1, PULSE_SHUTDOWN, 0);
        ConnectDetach(coid);
    }

    coid = ConnectAttach(ND_LOCAL_NODE, getpid(), chid_status, _NTO_SIDE_CHANNEL, 0);
    if (coid != -1) {
        MsgSendPulse(coid, -1, PULSE_SHUTDOWN, 0);
        ConnectDetach(coid);
    }

    pthread_join(sig_thread, NULL);
    pthread_join(stat_thread, NULL);

    ChannelDestroy(chid_signal);
    ChannelDestroy(chid_status);

    printf("[%s] Local Controller terminated cleanly.\n", intersection_name(id));
    return EXIT_SUCCESS;
}
