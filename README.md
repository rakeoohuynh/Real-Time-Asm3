# LC / CC proof-of-concept -- build & run notes

Three files:
- `common.h` -- shared message structs, timing constants (A1-A46) and
  the message set from Section 8's inter-task table.
- `local_controller.c` -- one Local Controller process (run once per
  intersection, e.g. `-i 1` for I1). Implements Phase_Controller_Task,
  Signal_Output_Task, Railway_Signal_Output_Task, Boom_Gate_Controller_Task,
  Status_Reporting_Task and Central_Command_Server_Task as described in
  Section 7, plus keyboard-simulated sensor/pedestrian/train input.
- `central_controller.c` -- the Central Controller process: a small
  server-thread pool for STATUS_UPDATE/FAULT_ALARM ingestion, a console
  dashboard, and an operator console for OVERRIDE_COMMAND / MODE_SWITCH.

## Build (QNX qcc, matches the qcc-terminal workflow already in use)

```
qcc -Vgcc_ntox86_64 -o local_controller   local_controller.c   -lpthread -lsocket
qcc -Vgcc_ntox86_64 -o central_controller central_controller.c -lpthread -lsocket
```

## Run -- same node (quick functional test)

```
./central_controller &
./local_controller -i 1
```

The LC looks for the CC's channel at `/dev/name/local/central_controller`
by default (same node). Type `t` + Enter in the LC's console to simulate
a train approaching, `p` for a pedestrian press, `n`/`e` for vehicle
demand on NS/EW, `c` for train cleared. In the CC console: `status`,
`override 1 allred`, `mode 1 sensor`.

## Run -- real QNET multi-node demo

```
# on the CC's node
./central_controller &

# on I1's node
./local_controller -i 1 -c <cc_node_name>

# on I2's node
./local_controller -i 2 -c <cc_node_name>
```

`-c` tells the LC which QNET node to find the CC on; the CC does not
need any equivalent flag -- it discovers each LC's node automatically
from the sender info on the first STATUS_UPDATE it receives, then
reuses that to open the OVERRIDE_COMMAND/MODE_SWITCH connection back.

## Where each requirement lands in the code

| Requirement | Where |
|---|---|
| LC operates continuously, independent of CC | `phase_controller_task()` runs on the LC's own thread/timer and never blocks on CC I/O |
| LC keeps running if the link/CC fails | `status_reporting_task()` + `send_report()` (UC-08 retry/back-off) |
| LC senses railway crossing, no control over it | `train_sensor_handle_approach/cleared()` -> `PULSE_TRAIN_APPROACH/CLEARED` -> `begin_rail_protection()` |
| Fixed-timing / sensor-driven / mode switch | `ctx->mode`, sensor-driven early-cut logic in `phase_controller_task()`, `NET_MODE_SWITCH` handling |
| Pedestrian buttons + signals | `pedestrian_input_handle_press()`, `begin_ped_walk()`, `STEP_PED_WALK/CLEARANCE` |
| LC->CC status on every state change | `notify_status()` calls sprinkled through every `begin_*`/`advance_*` function |
| CC displays intersections | `display_task()` / `status` operator command |
| CC override with LC-side safety check | `central_command_server_task()` validates before accepting, `apply_pending_cc_commands()` applies |
| Boom gate + fault reporting to control room | `boom_gate_task()`, `advance_rail_sequence()` FAULT_BOOM_GATE branch |

## Known PoC simplifications (documented, not hidden)

- STATUS_UPDATE/FAULT_ALARM are sent as a short blocking `MsgSend()`
  with an immediate empty reply rather than a true pulse, because a
  pulse's payload is too small (4 bytes) to carry the full status
  snapshot across a QNET connection -- see the comment in `common.h`.
  This keeps the "LC does not wait on CC" property in practice (the
  reply is immediate and the whole exchange runs on its own thread).
- Only I1's full railway-adjacent task set is exercised end-to-end;
  I2-I6 reuse the identical binary (`-i 2`, `-i 3`, ...) per the
  "condensed, same architecture" note in Section 7.
- `netmgr_ndtostr()`/`ND2S_LOCAL_STR` usage in `central_controller.c`
  should be checked against the exact SDP 7.1 header on your machine
  before the demo -- the flag name has moved slightly between QNX
  releases.git