/*
 * console_input.h -- keyboard stand-in for the sensors.
 *
 * Each key calls the handler of the task it simulates. Trains normally
 * come from the timetable (train_schedule.c); 't' and 'c' force one by
 * hand.
 */
#ifndef CONSOLE_INPUT_H
#define CONSOLE_INPUT_H

#include "lc_context.h"

void *console_input_task(void *arg);

#endif /* CONSOLE_INPUT_H */
