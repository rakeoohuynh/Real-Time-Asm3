/* =====================================================================
 * boom_gate.c -- Boom_Gate_Controller_Task (Section 8 rows
 * COMMAND_BOOM_GATE / GATE_STATUS).
 * ===================================================================== */
#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include "boom_gate.h"

/* PoC value with no report source: the gate is assumed to raise in half
 * the time it takes to lower, since raising is not safety-critical. */
#define GATE_RAISE_S   (RAIL_GATE_LOWER_S / 2)

/* How long the gate is given to complete its travel before the move is
 * declared a timeout fault. Lowering must finish inside the A16 window,
 * so the window is also the deadline. */
#define GATE_MOVE_TIMEOUT_S   RAIL_GATE_LOWER_S

static int g_close_attempts = 0;   /* attempts used in the current episode */

/* pulse payload packing: result | closing<<8 | gate_id<<16 */
#define GATE_PULSE_PACK(res, closing, id) \
    ( ((res) & 0xFF) | (((closing) & 1) << 8) | (((id) & 0xFF) << 16) )
#define GATE_PULSE_RESULT(v)   ((v) & 0xFF)
#define GATE_PULSE_CLOSING(v)  (((v) >> 8) & 1)
#define GATE_PULSE_ID(v)       (((v) >> 16) & 0xFF)

void *boom_gate_task(void *arg)
{
    lc_context_t *ctx = arg;
    int coid_phase = ctx->coid_phase;

    for (;;) {
        local_msg_t msg;
        int rcvid = MsgReceive(ctx->gate_chid, &msg, sizeof(msg), NULL);
        if (rcvid <= 0) continue;

        if (msg.type != MSG_COMMAND_BOOM_GATE) {
            local_reply_t bad = { .result = FAULT_CONFIG, .elapsed_ms = 0 };
            MsgReply(rcvid, EOK, &bad, sizeof(bad));
            continue;
        }

        int closing   = (msg.body.boom_gate.command == 1);
        int gate_id   = msg.body.boom_gate.gate_id;
        int timeout_s = msg.body.boom_gate.timeout_s;

        if (ctx->verbose) {
            printf("[I%d][Boom_Gate_Controller_Task] gate %d command=%s, timeout=%ds\n",
                   ctx->id, gate_id, closing ? "CLOSE" : "OPEN", timeout_s);
            fflush(stdout);
        }

        /* Accept the command and release the controller straight away;
         * the travel time is simulated below on this thread. */
        local_reply_t ack = { .result = 0, .elapsed_ms = 0 };
        MsgReply(rcvid, EOK, &ack, sizeof(ack));

        /* LC_FORCE_GATE_FAULT       -- every close attempt times out.
         * LC_FORCE_GATE_FAULT_ONCE  -- only the first attempt does, so a
         *                              demo can show the retry recovering.
         * 'g' key                   -- the next close attempt times out;
         *                              consumed here, so the retry succeeds. */
        int forced_always = closing && (getenv("LC_FORCE_GATE_FAULT") != NULL);
        int forced_once   = closing && (getenv("LC_FORCE_GATE_FAULT_ONCE") != NULL)
                                    && (g_close_attempts <= 1);
        int forced_key    = closing && lc_take_gate_fault(ctx);
        int forced        = forced_always || forced_once || forced_key;

        int travel_s = closing ? RAIL_GATE_LOWER_S : GATE_RAISE_S;
        if (forced) travel_s = timeout_s + 1;

        lc_delay_real_ms(travel_s * 1000);

        int result = (travel_s > timeout_s) ? FAULT_BOOM_GATE : 0;
        if (result != 0) {
            printf("[I%d][Boom_Gate_Controller_Task] gate %d FAULT (timeout after %ds)\n",
                   ctx->id, gate_id, travel_s);
            fflush(stdout);
        } else if (ctx->verbose) {
            printf("[I%d][Boom_Gate_Controller_Task] gate %d %s confirmed\n",
                   ctx->id, gate_id, closing ? "LOCKED" : "OPENED");
            fflush(stdout);
        }

        MsgSendPulse(coid_phase, SIGEV_PULSE_PRIO_INHERIT, PULSE_GATE_STATUS,
                     GATE_PULSE_PACK(result, closing, gate_id));
    }
    return NULL;
}

static void gate_command(lc_context_t *ctx, int command, int timeout_s)
{
    local_msg_t m; local_reply_t r;
    m.type = MSG_COMMAND_BOOM_GATE;
    m.body.boom_gate.gate_id   = BOOM_GATE_ID;
    m.body.boom_gate.command   = command;
    m.body.boom_gate.timeout_s = timeout_s;
    if (MsgSend(ctx->coid_gate, &m, sizeof(m), &r, sizeof(r)) == -1) {
        /* The gate task is unreachable: synthesise the fault locally so
         * the sequence still reaches its fault branch. */
        lc_raise_fault(ctx, FAULT_BOOM_GATE);
        lc_set_gate_state(ctx, GATE_TIMEOUT_FAULT);
        printf("[I%d][Boom_Gate_Controller_Task] gate command unreachable (errno path)\n", ctx->id);
        fflush(stdout);
    }
}

void boom_gate_begin_close(lc_context_t *ctx)
{
    g_close_attempts = 1;
    lc_set_gate_state(ctx, GATE_LOWERING);
    gate_command(ctx, 1 /*CLOSE*/, GATE_MOVE_TIMEOUT_S);
}

void boom_gate_retry_close(lc_context_t *ctx)
{
    g_close_attempts++;
    lc_set_gate_state(ctx, GATE_LOWERING);
    printf("[I%d][Boom_Gate_Controller_Task] retrying CLOSE (attempt %d of %d)\n",
           ctx->id, g_close_attempts, GATE_CLOSE_MAX_ATTEMPTS);
    fflush(stdout);
    gate_command(ctx, 1 /*CLOSE*/, GATE_MOVE_TIMEOUT_S);
}

void boom_gate_begin_open(lc_context_t *ctx)
{
    lc_set_gate_state(ctx, GATE_RAISING);
    gate_command(ctx, 0 /*OPEN*/, GATE_MOVE_TIMEOUT_S);
}

gate_event_t boom_gate_on_status(lc_context_t *ctx, int pulse_value)
{
    int result  = GATE_PULSE_RESULT(pulse_value);
    int closing = GATE_PULSE_CLOSING(pulse_value);

    if (!closing) {
        lc_set_gate_state(ctx, (result == 0) ? GATE_OPEN : GATE_TIMEOUT_FAULT);
        return GATE_EVT_OPENED;
    }

    if (result == 0) {
        lc_set_gate_state(ctx, GATE_LOCKED);
        lc_clear_fault(ctx, FAULT_BOOM_GATE);
        g_close_attempts = 0;
        return GATE_EVT_LOCKED;
    }

    lc_set_gate_state(ctx, GATE_TIMEOUT_FAULT);
    lc_raise_fault(ctx, FAULT_BOOM_GATE);
    return (g_close_attempts < GATE_CLOSE_MAX_ATTEMPTS)
               ? GATE_EVT_CLOSE_RETRY
               : GATE_EVT_CLOSE_FAILED;
}

int boom_gate_attempts_used(void) { return g_close_attempts; }
