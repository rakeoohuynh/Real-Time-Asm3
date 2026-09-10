/* =====================================================================
 * console_input.c -- simulated sensor input.
 * ===================================================================== */
#include <stdio.h>
#include "console_input.h"
#include "pedestrian.h"
#include "sensor_driven.h"
#include "railway_protection.h"

void *console_input_task(void *arg)
{
    lc_context_t *ctx = arg;
    printf("[I%d] keys: p=pedestrian  n=vehicle(NS)  e=vehicle(EW)  "
           "t=train approach (manual)  c=train cleared  q=quit\n", ctx->id);
    fflush(stdout);

    char line[16];
    while (fgets(line, sizeof(line), stdin)) {
        switch (line[0]) {
            case 'p': pedestrian_input_handle_press(ctx); break;
            case 'n': vehicle_sensor_handle_detect(ctx, 1); break;
            case 'e': vehicle_sensor_handle_detect(ctx, 0); break;
            case 't': train_sensor_handle_approach(ctx); break;
            case 'c': train_sensor_handle_cleared(ctx); break;
            case 'q': printf("[I%d] shutting down input task\n", ctx->id); return NULL;
            default: break;
        }
    }
    return NULL;
}
