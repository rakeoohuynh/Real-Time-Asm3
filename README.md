# Traffic light controller (LC/CC): build and run notes

Two QNX programs:

- **Local Controller (LC)**: one per intersection (`-i 1` for I1, and so on).
  It runs the signals, the pedestrian crossing, the right-turn arrows and
  railway protection for its intersection, and keeps working when the
  Central Controller or the network is down.
- **Central Controller (CC)**: collects status from every LC, shows a
  dashboard and sends operator commands (overrides and mode switches).

Source layout, under `asm3_sensor_driven/`:

- `shared/common.h`: message layouts, channel names and timing constants
  used by both programs.
- `lc/`: the Local Controller. `local_controller.c` is the entry point;
  the comment at its top lists which file holds each task.
- `cc/central_controller.c`: the Central Controller.

## Build

From `asm3_sensor_driven/`:

```
./build.sh                 # or: make -f Makefile.poc
```

or by hand:

```
qcc -Vgcc_ntox86_64 -Ishared -Ilc -o local_controller   lc/*.c                  -lpthread -lsocket
qcc -Vgcc_ntox86_64 -Ishared      -o central_controller cc/central_controller.c -lpthread -lsocket
```

If the linker reports `cannot find -lpthread`, drop that flag. On QNX 7.1
the pthread functions are part of libc.

## Run on one machine

```
./central_controller &
./local_controller -i 1
```

Without `-c`, the LC looks for the CC on the same node, at
`/dev/name/local/central_controller`. In the LC's console, type `t` and
Enter to send a train, `p` for a pedestrian button press, `n`/`e` for a
vehicle on NS/EW, and `c` to clear the train. In the CC's console, try
`status`, `override 1 allred` or `mode 1 sensor`.

## Run across machines (QNET)

On each VM, go to the folder with the binary and make it executable
(`central_controller` on the CC's node, `local_controller` on each LC's
node):

```
cd /tmp
chmod +x central_controller   # or: chmod +x local_controller
```

Start each program in the foreground. Don't use `&`, since both read
commands from the keyboard:

```
# on the CC's node
./central_controller

# on I1's node
./local_controller -i 1 -c <cc_node_name>

# on I2's node
./local_controller -i 2 -c <cc_node_name>
```

If the machines can't see each other (`ls /net` doesn't list the other
node), run this on each VM, then check `ls /net` again:

```
setconf _CS_DOMAIN net.intra
```

`-c` tells the LC which QNET node the CC is on. The CC needs no
equivalent: it learns each LC's node from its first status report and
sends commands back to that node.

## Central Controller console

The CC prints its command list at startup. Type one command per line:

| Command | What it does |
|---|---|
| `status` | Print the dashboard now (it's also printed every 10 s) |
| `override <id> <type>` | Send an OVERRIDE_COMMAND to intersection `<id>` |
| `mode <id> <fixed\|sensor>` | Send a MODE_SWITCH to intersection `<id>` |
| `verbose <on\|off>` | `on`: log every STATUS_UPDATE. `off` (default): log only first contact and changes of phase, mode or faults |
| `quit` | Leave the console |

The dashboard has one row per intersection:

```
======================== CC DASHBOARD 14:03:22 ========================
ID   PHASE         NS           EW           PED        GATE      MODE    LINK       FAULTS
I1   NS            GREEN+ARROW  RED          DONT_WALK  OPEN      SENSOR  OK 2s      none
I2   RAIL_PROTECT  RED          RED          DONT_WALK  FAULT     FIXED   STALE 31s  BOOM_GATE
```

`+ARROW` means that approach's right-turn arrow is green. `LINK` is the
time since that LC's last report, by the CC's clock. An LC only reports
when its state changes, so `LINK` switches to `STALE` only after
`2 x VEHICLE_GREEN_S / TIME_SCALE_FACTOR + 5` seconds (23 s at the
default factor of 5). FAULT_ALARMs are always logged, whatever the
verbose setting.

`<id>` is the number given to the LC's `-i` flag. Override types:

| Type | Effect on the LC |
|---|---|
| `allred` | Hold both approaches at red |
| `nsgreen` | Hold NS at green |
| `ewgreen` | Hold EW at green |
| `dignitary` | Clear a route for a dignitary; currently the same as `nsgreen` |

An override holds for `OVERRIDE_HOLD_S` (20 s, scaled by
`TIME_SCALE_FACTOR`), then the normal cycle resumes. You can send a
command to an intersection the CC hasn't heard from yet; it's delivered
once that LC's first report tells the CC where it is.

### When an override is refused

An override isn't guaranteed to take effect. The LC refuses it with
`NET_RESULT_WAIT`, a reason and a wait time while railway protection or
a pedestrian crossing is running. The wait covers the rest of that whole
sequence, not just the current step. The CC resends in the background
after the wait, up to 4 attempts in total, so the console stays usable.
A newer command of the same kind for the same intersection replaces a
queued one.

The LC also checks again just before applying an accepted override. If
railway protection or a crossing started in between, it drops the
override and logs `OVERRIDE_COMMAND ... DISCARDED`. MODE_SWITCH is always
accepted; it changes how later steps are timed, not the lights showing
now.

## Control mode by time of day

Each LC picks its own control mode from the time of day, so this works
even without the CC:

| Period | Hours | Mode |
|---|---|---|
| Peak | 06:30-09:00, 16:30-19:30 | fixed timing |
| Off-peak | 09:00-16:30, 19:30-22:00 | sensor-driven |
| Night | 22:00-06:30 | sensor-driven |

The time comes from the same simulated clock as the train timetable, so
`-T` moves both. For example, `-T 08:58` switches from FIXED to SENSOR
about 2 simulated minutes after start (24 real seconds at the default
factor of 5). The LC logs each switch:

```
[I1][Mode_Schedule] reached 09:00, OFF-PEAK period -> SENSOR mode
```

A CC `mode <id> <fixed|sensor>` takes effect right away and holds until
the next period change, when the schedule takes over again.

## Local Controller options and keys

```
./local_controller -i <id> [-c <cc_node>] [-T HH:MM] [-N] [-v]
```

| Option | Meaning |
|---|---|
| `-i <id>` | Intersection id, e.g. `-i 1` for I1. Also sets this LC's channel name, `lc_I<id>`, which the CC uses to send commands |
| `-c <node>` | QNET node the CC runs on. Leave it out when the CC is on the same node |
| `-T HH:MM` | Start the simulated clock at this time, e.g. `-T 06:28` for just before the morning peak. Trains and the control mode both follow it |
| `-N` | No train timetable; trains come only from the `t` key |
| `-v` | Also log every signal-head, railway-signal, right-turn-arrow and gate command |

Whenever the step, a signal, the gate, the mode or the faults change,
the LC prints one status line:

```
[I1 06:28:15] NS_GREEN         45s  NS:GREEN+ARROW EW:RED         PED:DONT_WALK GATE:OPEN     FIXED
[I1 06:29:02] RAIL_WARN        15s  NS:RED         EW:RED         PED:DONT_WALK GATE:OPEN     FIXED
```

The time is the machine's wall clock, the same one the CC uses, so the
two consoles line up. The number after the step is the real-world time
left in that step. `FAULT:<names>` is added while a fault is active.
Events such as railway protection, pedestrian requests, CC commands and
gate faults still get their own log lines; only the per-signal lines
need `-v`.

Keys (one per line):

| Key | Simulated event |
|---|---|
| `p` | Pedestrian button press |
| `n` | Vehicle detected on the NS approach |
| `e` | Vehicle detected on the EW approach |
| `t` | Train approaching; starts railway protection |
| `c` | Train cleared |
| `g` | The next gate close times out; the retry succeeds |
| `q` | Stop reading keys. The controller keeps running |

`n` and `e` only matter in sensor-driven mode, where a green with no
waiting traffic can end early. In fixed-timing mode every green runs its
full length. `t` and `c` work in both modes, since railway protection
takes priority over everything else.

`g` only arms the fault. It happens the next time the gate is told to
close, which is during railway protection after the crossing warning. To
see it, press `g`, then `t` (or wait for a scheduled train). The LC logs
`BOOM GATE FAULT`, turns the train light red and sends a FAULT_ALARM,
which the CC logs and shows under FAULTS. After `GATE_RETRY_INTERVAL_S`
the retry locks the gate and the fault clears. To make every attempt
fail instead, start the LC with `LC_FORCE_GATE_FAULT=1 ./local_controller ...`.

## Where each requirement is handled

| Requirement | Where |
|---|---|
| LC runs on its own, independent of the CC | `phase_controller_task()` runs on its own timer and never waits on CC I/O |
| LC keeps running if the link or CC fails | `status_reporting_task()` and `send_report()` (retry and back-off) |
| LC detects trains at the crossing | `train_sensor_handle_approach/cleared()` -> `PULSE_TRAIN_APPROACH/CLEARED` -> `railway_begin_protection()` |
| Fixed-timing and sensor-driven modes, mode switch | `ctx->mode`, `mode_schedule.c`, `sensor_driven_tick()`, MODE_SWITCH handling in `apply_pending_cc_commands()` |
| Pedestrian buttons and signals | `pedestrian_input_handle_press()`, `pedestrian_begin_walk()`, `STEP_PED_WALK/CLEARANCE` |
| LC reports every state change to the CC | `notify_status()` calls in each `begin_*`/`advance_*` function |
| CC shows the intersections | `display_task()` and the `status` command |
| CC override with an LC-side safety check | `central_command_server_task()` checks before accepting; `apply_pending_cc_commands()` checks again and applies |
| Boom gate and fault reporting | `boom_gate_task()`, `railway_on_gate_status()`, `enter_gate_fault()` |

## Startup connection check

Each `local_controller` run starts with `probe_cc_connectivity()`. It
tries to open the CC's channel up to 5 times, 1 s apart, and prints the
result before anything else starts:

```
[I2] checking connection to CC (node: machineA) ...
[I2] CC REACHABLE (attempt 1/5) -- opened channel 'central_controller' over QNET
```

or, if QNET isn't set up yet:

```
[I2] *** CC NOT REACHABLE after 5 attempts ***
     Check that the CC is running, both machines are on the same network,
     QNET (lsm-qnet.so) is running on both, and the node name 'machineA' is right:
     `ls /net/machineA/dev/name/local/` on this machine should list the CC's channel.
     The LC starts anyway and keeps trying to reach the CC in the background.
```

The check is only a diagnostic: the LC starts and runs either way. A
failure tells you straight away that the problem is the network or QNET
setup, not the control logic.

## Known limitations

- STATUS_UPDATE and FAULT_ALARM go to the CC as a short `MsgSend()`
  rather than a pulse, because a pulse only carries 4 bytes. The CC
  replies immediately and the send runs on its own thread, so the LC
  isn't held up in practice. See the comment in `common.h`.
- Every intersection runs the same binary with a different `-i`,
  including the railway crossing logic.
- `netmgr_ndtostr()` and `ND2S_LOCAL_STR` in `central_controller.c` have
  changed between QNX releases. If the CC doesn't build, check
  `<sys/netmgr.h>` in your SDP.
