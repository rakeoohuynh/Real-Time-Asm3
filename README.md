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

On each VM, go to the folder holding the binary and make it executable
(`central_controller` on the CC's node, `local_controller` on each LC's
node):

```
cd /tmp
chmod +x central_controller   # or: chmod +x local_controller
```

Then start each process in the foreground (no `&` -- both read commands
from stdin):

```
# on the CC's node
./central_controller

# on I1's node
./local_controller -i 1 -c <cc_node_name>

# on I2's node
./local_controller -i 2 -c <cc_node_name>
```

If the machines can't see each other (`ls /net` does not list the other
node), run this on each VM, then check `ls /net` again:

```
setconf _CS_DOMAIN net.intra
```

`-c` tells the LC which QNET node to find the CC on; the CC does not
need any equivalent flag -- it discovers each LC's node automatically
from the sender info on the first STATUS_UPDATE it receives, then
reuses that to open the OVERRIDE_COMMAND/MODE_SWITCH connection back.

## Central Controller -- operator console commands

The CC prints its command list on startup. One command per line on stdin:

| Command | Effect |
|---|---|
| `status` | Print the dashboard now (it is also printed every 10 s) |
| `override <id> <type>` | Send an OVERRIDE_COMMAND to intersection `<id>` |
| `mode <id> <fixed\|sensor>` | Send a MODE_SWITCH to intersection `<id>` |
| `verbose <on\|off>` | `on`: log every STATUS_UPDATE. `off` (default): log only first contact and changes of phase, mode or faults |
| `quit` | Leave the operator console |

The dashboard has one row per intersection:

```
======================== CC DASHBOARD 14:03:22 ========================
ID   PHASE         NS           EW           PED        GATE      MODE    LINK       FAULTS
I1   NS            GREEN+ARROW  RED          DONT_WALK  OPEN      SENSOR  OK 2s      none
I2   RAIL_PROTECT  RED          RED          DONT_WALK  FAULT     FIXED   STALE 31s  BOOM_GATE
```

`+ARROW` means that approach's right-turn arrow is green. `LINK` is the
time since that LC's last report, measured on the CC's clock. It turns
`STALE` after `2 x VEHICLE_GREEN_S / TIME_SCALE_FACTOR + 5` seconds
(23 s at the default factor of 5), since an LC reports only when its
state changes. A `FAULT_ALARM` is always logged, whatever the verbose
setting.

`<id>` is the number passed to the LC's `-i` flag. The four override types:

| Type | Effect on the LC |
|---|---|
| `allred` | Force both approaches to RED and hold |
| `nsgreen` | Force the NS approach green and hold |
| `ewgreen` | Force the EW approach green and hold |
| `dignitary` | Dignitary path; currently the same forced-NS-green sequence |

A forced state is held for `OVERRIDE_HOLD_S` (20 s, scaled by
`TIME_SCALE_FACTOR`) before the normal cycle resumes. An intersection does
not have to be known to the CC yet: the registry entry is created on
demand, though the command can only be delivered once that LC's first
STATUS_UPDATE has revealed its node.

### When an override is refused

The LC validates every override twice, so `override` is not guaranteed to
take effect. `Central_Command_Server_Task` replies `NET_RESULT_WAIT` with a
reason and an estimated wait if railway protection is active or a
pedestrian is mid-crossing. The wait covers the rest of the whole railway
or pedestrian sequence, not just the current step. The CC's command worker
resends after that wait (up to 4 attempts in total) in the background, so
the operator console stays usable; a newer command of the same kind for the
same intersection replaces a queued one. If either condition starts in the gap between
acceptance and application, `apply_pending_cc_commands()` discards the
command and prints `OVERRIDE_COMMAND ... DISCARDED`. MODE_SWITCH is always
accepted; it changes sequencing policy, not the current signal state.

## Local Controller -- flags and keys

```
./local_controller -i <id> [-c <cc_node>] [-T HH:MM] [-N] [-v]
```

| Flag | Meaning |
|---|---|
| `-i <id>` | Intersection id, e.g. `-i 1` for I1. Also fixes this LC's own channel name, `lc_I<id>`, which the CC opens to send commands back |
| `-c <node>` | QNET node the CC runs on. Omit for same-node testing |
| `-T HH:MM` | Seed the simulated clock, e.g. `-T 06:28` to start just before a morning peak train |
| `-N` | Do not run the train timetable; trains then arrive only via the `t` key |
| `-v` | Verbose: also log every signal-head, railway-signal, right-turn-arrow and boom-gate command |

Whenever the step, a signal head, the gate, the mode or the faults
change, the LC prints one status line:

```
[I1 06:28:15] NS_GREEN         45s  NS:GREEN+ARROW EW:RED         PED:DONT_WALK GATE:OPEN     FIXED
[I1 06:29:02] RAIL_WARN        15s  NS:RED         EW:RED         PED:DONT_WALK GATE:OPEN     FIXED
```

The time is the VM's wall clock, the same clock the CC stamps its log
with, so the LC and CC consoles line up. The number after the step is
the real-world seconds left in it. The train timetable (`-T`) still runs
on its own accelerated clock (A40). `FAULT:<names>` is appended
when a fault is active. Events such as railway protection, pedestrian
requests, CC commands and gate faults are still logged on their own
lines. Only the per-head output lines need `-v`.

The LC prints its key list on startup. Each key is one line on stdin:

| Key | Simulated event |
|---|---|
| `p` | Pedestrian button press |
| `n` | Vehicle detected on the NS approach |
| `e` | Vehicle detected on the EW approach |
| `t` | Train approaching; starts railway protection |
| `c` | Train cleared; ends railway protection |
| `q` | Stop the input task. The control tasks keep running |

`n` and `e` only change the outcome in sensor-driven mode, where absent
demand lets the current green be cut early. In fixed-timing mode the cycle
runs to its full length regardless. `t` and `c` work in both modes, since
railway protection outranks everything else.

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

## Startup connectivity self-check

Every `local_controller` run begins with `probe_cc_connectivity()`: it
tries `name_open()` to the CC's channel up to 5 times (1s apart) and
prints a clear `CC REACHABLE` / `CC NOT REACHABLE` verdict before
anything else starts, e.g.:

```
[I2] checking connectivity to CC (target node: machineA) ...
[I2] CC REACHABLE (attempt 1/5) -- channel 'central_controller' resolved OK over QNET
```

or, if WiFi/QNET isn't wired up yet:

```
[I2] *** CC NOT REACHABLE after 5 attempts ***
     Check: CC process running? Same WiFi/subnet? QNET (io-pkt + npm-qnet.so)
     mounted on both machines? Node name 'machineA' correct (try `ls /net/machineA/dev/name/local/`
     from a shell on this machine)? -- LC will still start and operate autonomously;
     Status_Reporting_Task keeps retrying in the background (UC-08).
```

This is diagnostic only -- the LC starts and runs normally either way
(matching the "operate autonomously if the link fails" requirement);
a failed probe just tells you immediately that it's a network/QNET
setup issue rather than something wrong in the control logic, so you
can fix it before the demo instead of during it.

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
  releases.