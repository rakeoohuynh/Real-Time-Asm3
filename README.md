# Traffic Light Controller System — Phase 1 (I1, I2, `fixed_time`)

EEET2588/EEET2687 Real-Time Systems Engineering — Assessment 3 (Design Project),
Team NTC. QNX SDP 7.1 / Momentics, C.

This phase implements **intersections I1 and I2, operating in `fixed_time` mode
only**, as a faithful, reduced-scope coding implementation of the team's
Assessment 2 Initial Design report ("EEET2588_Group NTC_Asm2.pdf").
It does not implement sensor-driven mode, mode switching, pedestrian crossings,
central override, or the railway crossing subsystem — see **Scope** below for
why.

> **Multi-PC bridged network test:** for the mandatory cross-computer Qnet
> verification (three physical PCs, bridged networking, no NAT/host-only), see
> [`NETWORK_TESTING.md`](NETWORK_TESTING.md) — a separate procedure that
> doesn't require any code changes.

---

## 1. Scope

**Implemented this phase:**
- Intersections I1 and I2, each an independent QNX process (Local Controller).
- `fixed_time` operation only — UC-01 (Fixed-Timing Operation) from Assessment 2.
- Status reporting to a Central Controller — UC-06, reduced to what UC-01's
  main flow (step 9) actually requires: a status push after every phase
  transition. The CC purely monitors/displays; it never issues commands in
  this phase.
- Full QNX native IPC: blocking Send/Receive/Reply for safety-critical signal
  commands, non-blocking pulses for status telemetry, a POSIX periodic timer
  for phase timing, and pthreads for task concurrency within each process.

**Deliberately not implemented this phase** (each is a distinct Assessment 2
use case, out of scope for "I1, I2, fixed_time only"):
- UC-02 Sensor-driven operation, UC-03 Mode switch command — no `SENSOR_DRIVEN`
  mode exists yet, so there is nothing to switch to; `phase_controller.c` is
  structured so a second mode and a `MODE_SWITCH` handler can be added later
  without changing the fixed-timing state machine itself.
- UC-04 Pedestrian crossing request — no pedestrian buttons/signal heads.
- UC-05 Train approach / railway crossing activation — no boom gate, train
  sensor, or railway signal. **Note:** the assessment brief's stated Pass-grade
  minimum is "I1-I2 **and** the railway crossing system." This phase covers
  I1/I2 only, by explicit direction for this iteration; the railway subsystem
  is intended for a follow-up phase, not silently dropped.
- UC-07 Central override command, UC-08 Central Controller communication loss
  — a **simplified** version of UC-08's autonomy requirement (A34: "the LC
  continues local safety-critical operation when CC communication is
  unavailable") is implemented, because it's a basic safety property of UC-01
  itself, not because UC-08 is in scope. See §4.3.

---

## 2. Architecture mapping

Component → Responsibility → Process/Thread → IPC, reduced from Assessment 2
Fig. 18 to what this phase needs:

| Component | Responsibility | Process / Thread | IPC / Synchronisation |
|---|---|---|---|
| **Phase_Controller_Task** | Safety-critical core FSM; drives the fixed NS→EW vehicle-phase cycle | LC process, **main thread** | Owns a QNX channel + 100 ms POSIX periodic timer (`timer_create`/`SIGEV_PULSE`); receives via `MsgReceive` |
| **Signal_Output_Task** | Confirms/"drives" the physical NS/EW signal heads | LC process, **pthread** | Own channel; blocking `MsgReceive`/`MsgReply` server for `SET_VEHICLE` |
| **Status_Reporting_Task** | Forwards phase-transition events to the CC without blocking the FSM; tolerates CC being offline | LC process, **pthread** | Own channel; receives `STATUS` via `MsgSendPulse` (from Phase_Controller_Task), forwards `STATUS_UPDATE` via `MsgSendPulse` (to CC) |
| **Central Controller** | Displays received status; does not control lights in this phase | Separate `cc` process | Local named channel (`name_attach`, flags=0); reached cross-node via Qnet's `/net/<node>/...` path; `MsgReceive` loop |

Two intersections ⇒ two independent `lc` processes (`lc I1 ...` and
`lc I2 ...`), each with the three-task layout above, matching Assessment 2
Section 7's statement that I2–I6 reuse I1's task architecture unchanged.

### 2.1 State machine

Exactly Assessment 2 Fig. 4 (Vehicle signal head state diagram), six states:

```
NS_GREEN(45s) → NS_YELLOW(3s) → ALL_RED(2s) → EW_GREEN(45s) → EW_YELLOW(3s) → ALL_RED(2s) → (repeat)
```

Durations are the PoC values from Assessment 2 Table 1 (A1–A4), scaled by
`DEMO_SCALE` in [`common/timing.h`](common/timing.h) for demonstration speed
(default 10× faster; set to 1 for the real 100 s cycle — see A40, which
explicitly permits this).

### 2.2 Message/pulse summary (from Assessment 2 §8, scoped to UC-01/UC-06)

| Message | Sender → Receiver | Primitive | Purpose |
|---|---|---|---|
| `PULSE_TIMER_TICK` | POSIX timer → Phase_Controller_Task (self) | `MsgSendPulse` (kernel) | 100 ms elapsed-time tick (A39) |
| `SET_VEHICLE` | Phase_Controller_Task → Signal_Output_Task | `MsgSend` (blocking) / `MsgReply` | Command + confirm a phase's NS/EW signal state |
| `STATUS` | Phase_Controller_Task → Status_Reporting_Task | `MsgSendPulse` (non-blocking) | Notify of a phase transition, without blocking the FSM |
| `STATUS_UPDATE` | Status_Reporting_Task → Central Controller | `MsgSendPulse` (non-blocking) | Push status to the control room display |
| `PULSE_SHUTDOWN` | main thread → Signal_Output_Task / Status_Reporting_Task | `MsgSendPulse` | Clean, ordered thread termination on SIGINT/SIGTERM |

---

## 3. Lab traceability

Every non-trivial mechanism below is the same technique demonstrated in a
specific lab, applied to this project's own state/timing/message values —
nothing is invented from scratch.

| Mechanism | Used in | Lab |
|---|---|---|
| `enum` + `switch` single-step state machine (`phase_next`, `phase_get_config`) | `lc/phase_controller.c` | Lab 5, Task 2A (*Implementing Traffic Lights Using a State Machine*) |
| Periodic POSIX timer → `ChannelCreate`/`ConnectAttach(self)`/`timer_create(SIGEV_PULSE)`/`timer_settime`/`MsgReceive` loop | `lc/phase_controller.c` | Lab 5, Task 3A (*Variable-Timing state machine using QNX POSIX Timers*) |
| Local (same-node) `ConnectAttach(ND_LOCAL_NODE, pid, chid, _NTO_SIDE_CHANNEL, 0)` between tasks in one process | `lc/phase_controller.c`, `lc/lc_main.c` | Lab 6, Task 2A (*Local message passing using PID + Channel ID*) |
| Blocking `MsgSend`/`MsgReceive`/`MsgReply` handshake | `lc/phase_controller.c` ↔ `lc/signal_output.c` | Lab 6 (Send/Receive/Reply) |
| Local named attach point (`name_attach`, flags=0) + `name_open`, reachable over Qnet from another node via `/net/<node>/...` (no `gns` service required) | `cc/cc_main.c`, `lc/status_reporting.c` | Lab 6, Topics 2–3 (Named connections; Qnet) |
| `pthread_create`/`pthread_join` for task concurrency within one process | `lc/lc_main.c` | Lab 2 |
| Error checking on every system/IPC call used | throughout | Course-wide requirement (Labs 1–6) |

---

## 4. Design notes

### 4.1 Why a 100 ms tick instead of one-shot re-armed timers

Lab 5 Task 3A re-arms `timer_settime()` with a *new* duration on every
transition (the timer fires exactly at state end). This project instead uses
one **periodic, never-reconfigured** 100 ms timer and an `elapsed_ms` counter
compared against the current phase's duration. This is a direct implementation
of Assessment 2 assumption **A39** ("100 ms periodic POSIX timer tick... for
elapsed-time monitoring; does not imply signals change every 100 ms") and
keeps `Phase_Controller_Task`'s `MsgReceive()` loop structurally ready for
later phases to add other event types (pedestrian requests, mode switches) on
the same channel without restructuring the timer.

### 4.2 Safety interlocks implemented

- The phase table (`phase_get_config`) can never produce NS=GREEN and
  EW=GREEN together, by construction (Assessment 2 **A43**).
- `Signal_Output_Task` independently re-checks and refuses any conflicting
  command it is ever sent (defence in depth), NACK-ing rather than displaying
  it (`lc/signal_output.c`).
- If `Signal_Output_Task` ever fails to confirm a commanded output,
  `Phase_Controller_Task` halts the cycle rather than advancing on an
  unconfirmed/unsafe state (Assessment 2 **A44** — safety precedence over
  ordinary operation).

### 4.3 Central Controller availability (simplified UC-08)

`Status_Reporting_Task` never blocks `Phase_Controller_Task` — status is
delivered by `MsgSendPulse`, which is inherently non-blocking on the sender's
side. If the CC is unreachable at startup or drops mid-run, the LC keeps
running its fixed-timing cycle unaffected (Assessment 2 **A34**); the task
retries the CC connection quietly every 10 status events. This is a scoped-down
version of UC-08's full retry-counter/backoff procedure — sufficient to
demonstrate the required autonomy property without importing UC-08's complete
design for a phase that doesn't otherwise touch central communication.

### 4.4 Why no mutex/semaphore/condition variable

Each task (`Phase_Controller_Task`, `Signal_Output_Task`,
`Status_Reporting_Task`) owns its state exclusively and never touches another
task's memory directly — all coordination between them is through QNX message
passing (`MsgSend`/`MsgReceive`/`MsgReply`, `MsgSendPulse`), not shared
variables. With nothing shared, there is no critical section and therefore no
mutex/semaphore/condvar to add. This matches Assessment 2 Section 7
verbatim: *"No additional shared-memory, mutex, semaphore, or
condition-variable mechanism is shown unless required by the
implementation"* — none is required here.

### 4.5 Extensibility

`phase_controller.h`/`.c` expose the vehicle-phase FSM as data (a transition
table + a config lookup) plus a `MsgReceive`-driven loop that already
tolerates unrecognised pulse codes. Adding `SENSOR_DRIVEN` mode or
`MODE_SWITCH` handling later means adding new pulse/message types to the same
loop, not restructuring `fixed_time`'s logic — matching the "later phases can
replace `fixed_time` without rewriting the entire program" goal.

---

## 5. Project structure

```
TrafficControllerSystem/
├── README.md
├── Makefile                    Builds bin/lc and bin/cc via qcc
├── common/
│   ├── protocol.h / .c         Wire format shared by LC and CC: IDs, phase
│   │                           enum, pulse codes, SET_VEHICLE message
│   └── timing.h                Fixed-timing durations (Assessment 2 A1-A4)
├── lc/                         Local Controller (one executable, run once
│   │                           per intersection)
│   ├── lc_main.c               Process entry point: creates channels, spawns
│   │                           the two worker threads, runs the FSM, shuts
│   │                           down cleanly
│   ├── phase_controller.h/.c   Phase_Controller_Task: the FSM (Lab 5) + timer
│   │                           (Lab 5) + IPC to the other two tasks (Lab 6)
│   ├── signal_output.h/.c      Signal_Output_Task: confirms signal-head
│   │                           commands (Lab 6 Send/Receive/Reply server)
│   └── status_reporting.h/.c   Status_Reporting_Task: forwards status to the
│                               CC without blocking the FSM (Lab 6 + Lab 2)
└── cc/
    └── cc_main.c                Central Controller: global name-attach point
                                 that displays received STATUS_UPDATE pulses
```

---

## 6. VM configuration — do I need `vm3`?

**NO.** `vm1` and `vm2` are sufficient for this phase.

- **`vm1`** runs `bin/lc I1` **and** `bin/cc`.
- **`vm2`** runs `bin/lc I2 /net/vm1/dev/name/local/traffic_cc_status`.

Reasoning:
- Each intersection's Local Controller is safety-critical and must be its own
  QNX process/node — that's the one hard node requirement, satisfied by
  `vm1` (I1) and `vm2` (I2).
- The Central Controller is not safety-critical (Assessment 2 §7: "the CC...
  never directly controls physical signal outputs") and has no reason to need
  a dedicated node. Running it alongside I1 on `vm1` is a deliberate choice,
  not a shortcut: it means **I1 talks to the CC locally** and **I2 talks to
  the CC over Qnet** — i.e., this two-VM setup already exercises *both* IPC
  paths the full six-intersection design would need (same-node and
  cross-node native QNX messaging), without a third target.
- If a third VM becomes genuinely necessary in a later phase (e.g. an
  isolated railway-crossing node), add it then — see current assessment
  Section: "prefer the minimum number of VMs necessary."

---

## 7. Building in QNX Momentics

### A. Import the project
`File → Import → General → Existing Projects into Workspace`, select this
`TrafficControllerSystem` folder. Momentics will pick up the `Makefile`
automatically (or create a new **QNX C Project (Makefile)** pointed at this
directory if you prefer a fresh project shell around the existing sources).

### B. Configure the target
Two Run/Debug configurations, one per binary:
- **`lc`** → deploy/run on **both** `vm1` (as `I1`) and `vm2` (as `I2`).
- **`cc`** → deploy/run on **`vm1`** only.

### C. Build
`Project → Build Project` (or `Project → Build All`). This invokes
`make all`, producing `bin/lc` and `bin/cc`.

Command-line equivalent (from a QNX Momentics shell with the QNX toolchain on
`PATH`):
```bash
make            # builds bin/lc and bin/cc for x86_64 targets
make CPU=aarch64le   # example: build for an ARM64 QNX target instead
```

### D. Deploy
Use Momentics' target file system view (or `scp`/QNX's target upload) to copy
`bin/lc` to `vm1` and `vm2`, and `bin/cc` to `vm1`. A shared filesystem export
between the VMs also works if your lab setup provides one.

### E. Run

On **`vm1`** (start the CC first, so I1 can connect immediately):
```bash
./cc &
./lc I1
```

On **`vm2`**:
```bash
./lc I2 /net/vm1/dev/name/local/traffic_cc_status
```

(Replace `vm1` with vm1's actual QNX node name if different — check with
`on -n vm1 hostname` or your lab's node-naming convention.)

### F. Observe output
- Each `lc` console prints its initial phase, every `PHASE -> PHASE`
  transition, and the confirmed `SIGNAL HEADS -> NS:... EW:...` line from
  `Signal_Output_Task` — this is the proof the state machine and fixed timing
  are both running correctly and safely (never two greens at once).
- The `cc` console prints a timestamped `STATUS_UPDATE` line per transition,
  from **both** I1 and I2, proving cross-process/cross-node IPC and
  synchronisation are functioning.

### G. Stop and clean up
`Ctrl+C` (SIGINT) in each console. Every process prints `... terminated
cleanly.` after joining its threads and destroying its channels/timer — see
Test 7 below. `make clean` removes `bin/`.

---

## 8. Test plan

| Test ID | Purpose | Command / Input | Expected Output | Proves |
|---|---|---|---|---|
| **T1 — Startup** | I1 and I2 initialise correctly | Start `cc`, then `lc I1`, then `lc I2 ...` | Each `lc` prints its banner, `Signal_Output_Task ready.`, `Status_Reporting_Task connected...` (or the autonomous-startup message if `cc` isn't up yet), then `Initial phase: NS_GREEN` and a confirmed `SIGNAL HEADS -> NS:GREEN EW:RED` line | I1 and I2 both initialise and enter the correct, safe initial state |
| **T2 — Fixed-time transition** | A phase changes after its configured duration | Let `lc I1` run past one `GREEN_MS` interval (~4.5 s at default `DEMO_SCALE`) | `[I1] NS_GREEN -> NS_YELLOW` followed by a confirmed `SIGNAL HEADS -> NS:YELLOW EW:RED` line, timed to the configured duration | Fixed timing is respected and confirmed before advancing |
| **T3 — Complete cycle** | One full 6-phase cycle completes and returns to the start | Let `lc I1` run for one full cycle (~10 s at default scale) | Sequence `NS_GREEN→NS_YELLOW→ALL_RED→EW_GREEN→EW_YELLOW→ALL_RED→NS_GREEN`, each with a confirmed SIGNAL HEADS line, no repeats/skips | The state machine implements the exact Assessment 2 Fig. 4 sequence |
| **T4 — Multiple cycles** | Behaviour repeats deterministically | Let `lc I1` run for 3+ full cycles | Same sequence each cycle, same durations each time | Determinism (Assessment 2 A45) |
| **T5 — I1 and I2 concurrently** | Both intersections run correctly at the same time | Run `lc I1` and `lc I2 ...` together | Interleaved but independent `[I1]`/`[I2]` transition logs, each following its own correct sequence and timing | Two independent QNX processes/nodes operate concurrently without interference |
| **T6 — IPC / synchronisation** | The commanded IPC/synchronisation primitives actually function | Run `cc`, `lc I1`, `lc I2 ...` together; watch the `cc` console | A `STATUS_UPDATE` line appears on `cc` for every `PHASE -> PHASE` transition from **both** I1 (local IPC) and I2 (Qnet IPC) | `MsgSend`/`MsgReceive`/`MsgReply`, `MsgSendPulse`, and the POSIX timer are all functioning end-to-end, including over the network |
| **T6b — CC unavailable** | LC operates autonomously without the CC | Start `lc I1` **before** starting `cc` | `[I1] Status_Reporting_Task: Central Controller unavailable at startup - operating autonomously...`, followed by the normal fixed-timing cycle continuing unaffected; once `cc` is started, `... link RESTORED.` appears within ~10 status events | Assessment 2 A34: the LC never depends on CC availability for safety-critical operation |
| **T7 — Termination** | Clean shutdown, no leaked processes/threads/resources | `Ctrl+C` on each running process | Each process prints its two worker-thread shutdown lines and finally `... Local Controller terminated cleanly.` / `Central Controller terminated cleanly.`; `pidin` (or `ps`) shows no leftover processes afterward | Threads are joined, channels/timers destroyed, and processes exit deliberately rather than being killed |

---

## 9. Requirements traceability matrix

| Requirement | Source | Implementation | Evidence / Test |
|---|---|---|---|
| I1 implemented | Current assessment + custom scope prompt | `lc I1` process (`lc/lc_main.c` + full task set) | T1, T3, T5 |
| I2 implemented | Current assessment + custom scope prompt | `lc I2` process (same binary, `argv[1]="I2"`) | T1, T3, T5 |
| `fixed_time` mode | Current assessment; Assessment 2 UC-01 | `lc/phase_controller.c` (`phase_next`, `phase_get_config`) | T2, T3, T4 |
| State machine (enum + switch) | Assessment 2 Fig. 4 / Lab 5 Task 2A | `phase_next()`, `phase_get_config()` | T3, T4 |
| Deterministic timing (100 ms tick) | Assessment 2 A39 / Lab 5 Task 3A | `phase_controller_run()` timer setup + loop | T2 |
| IPC: blocking Send/Receive/Reply | Assessment 2 §8; current assessment (mandatory QNX IPC) | `SET_VEHICLE` in `phase_controller.c` ↔ `signal_output.c` | T1, T2, T6 |
| IPC: non-blocking pulses | Assessment 2 §8 | `STATUS` / `STATUS_UPDATE` pulses | T6 |
| IPC: named + Qnet cross-node connection | Current assessment ("multiple QNX nodes"); Lab 6 | `cc_main.c` (`name_attach`, local) + `status_reporting.c` (`name_open` via `/net/<node>/...`) | T6 |
| Synchronisation: pthreads | Lab 2; Assessment 2 Fig. 18 | `pthread_create`/`pthread_join` in `lc_main.c` | T1, T7 |
| Safety: no conflicting greens | Assessment 2 A43 | Phase table construction + `signal_output.c` interlock check | T3 (inspection of output) |
| Safety: fail-safe on unconfirmed output | Assessment 2 A44 | `phase_controller_run()` halts cycle on `send_set_vehicle` failure | Code inspection / fault injection (not exercised by default demo) |
| Autonomy without CC | Assessment 2 A34 (simplified from UC-08) | `status_reporting.c` reconnect logic | T6b |
| Demonstrable timing (accelerated) | Assessment 2 A40; current assessment ("timing... may be sped up") | `DEMO_SCALE` in `common/timing.h` | T2, T3 |
| Clean termination | Section 16 (Testing Requirements) | SIGINT/SIGTERM handling + `pthread_join` + resource cleanup in `lc_main.c`/`cc_main.c` | T7 |
| Railway crossing system | Current assessment (stated Pass-grade minimum) | **Not implemented this phase** — by explicit scope decision, see §1 | — |

---

## 10. Troubleshooting

- **`ERROR: name_attach('traffic_cc_status') failed: No such file or directory`**
  — `cc` isn't running into a name collision; this specific error means
  something else already exists at that path, or (if you re-add
  `NAME_FLAG_ATTACH_GLOBAL`) that the Global Name Service (`gns`) isn't
  running on this node. The shipped code uses a local attach point
  precisely to avoid needing `gns` on a minimal `mkqnximage` build — no
  extra service required.
- **`ERROR: name_attach('traffic_cc_status') failed: File exists`** (or
  similar "already in use") — another `cc` instance is already running (or a
  stale one didn't shut down cleanly); stop it first, or reboot the target's
  QNX session.
- **`lc` prints "Central Controller unavailable at startup"** — this is
  expected if `cc` hasn't started yet, or if the QNET path/node name passed as
  `argv[2]` is wrong; the LC still runs correctly (T6b). Verify the node name
  matches your lab's actual VM hostname.
- **Build fails with "qcc: not found"** — the QNX Momentics toolchain isn't on
  `PATH`; build from within Momentics (`Project → Build Project`) instead, or
  source the QNX SDP environment script before running `make`.
- **No `SIGNAL HEADS` lines appear** — check that `Signal_Output_Task ready.`
  printed at startup; if `pthread_create` failed the process would have exited
  immediately with an error message.
