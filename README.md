# SKY130WORKSHOP-PROJECT
Ultrasonic Distance Meter and Proximity Alarm Controller for Obstacle Detection
# Ultrasonic Distance Meter and Proximity Alarm Controller for Obstacle Detection

A small, synthesizable Verilog block that talks to an HC-SR04 style ultrasonic sensor on its own. It generates the trigger pulse, times the echo, converts the echo width to centimetres **without a divider**, and raises an alarm when an object is closer than a programmable threshold. No microcontroller is needed.

| | |
|---|---|
| Module name | `ultrasonic_ranger` |
| Technology target | SkyWater sky130 (`sky130_fd_sc_hd`) |
| Clock | 10 MHz (100 ns period) |
| Language | Verilog-2005 (synthesizable) |
| Tools used | Icarus Verilog, GTKWave, Yosys |

---

## Table of contents

1. [Abstract](#1-abstract)
2. [Real-world use and problem solved](#2-real-world-use-and-problem-solved)
3. [Architecture and working principle](#3-architecture-and-working-principle)
4. [Interface (inputs and outputs)](#4-interface-inputs-and-outputs)
5. [Design parameters and the scaled simulation](#5-design-parameters-and-the-scaled-simulation)
6. [Verification plan](#6-verification-plan)
7. [Project status](#7-project-status)
8. [Repository structure](#8-repository-structure)
9. [Execution on the virtual machine, with results explained](#9-execution-on-the-virtual-machine-with-results-explained)
10. [Results summary](#10-results-summary)
11. [Challenges faced and how they were solved](#11-challenges-faced-and-how-they-were-solved)


---

## 1. Abstract

Ultrasonic sensors such as the HC-SR04 report distance as the width of an echo pulse, about 58 microseconds per centimetre of distance. Measuring this normally needs a microcontroller running timing code, which adds cost and power, and software or interrupt latency adds jitter to the reading.

This project implements the whole measurement as a compact digital circuit. A 6-state finite state machine sequences the measurement cycle, a 20-bit timer handles the gap, trigger and timeout intervals, and a tick counter turns elapsed clock cycles into whole centimetres, so no division hardware is required. The design also detects faults (no echo, out-of-range echo) and compares the result against an 8-bit threshold to drive a proximity alarm output.

The design was verified with a self-checking testbench (50 checks, all passing) using a behavioural sensor model, synthesized with Yosys against the sky130 standard-cell library (59 flip-flops, about 2840 µm²), and verified again at gate level, where the synthesized netlist passed the same 50 checks and produced waveforms matching the RTL.

---

## 2. Real-world use and problem solved

**Problem.** A distance reading from an ultrasonic sensor is only a pulse width. Turning it into a number with exact timing normally requires a processor, which is expensive and power-hungry for a simple sensing node, and software timing is not deterministic.

**Solution.** A dedicated hardware block placed next to the sensor does the whole job with one-clock-cycle timing resolution and no software.

**Where it would be used.**

| Application | Role of the circuit |
|---|---|
| Car parking-assist sensors | Measures distance to an obstacle and drives a beeper through `alarm` |
| Obstacle-avoiding robots | Gives a fast distance reading and a "too close" flag |
| Blind-spot and proximity warning | Always-on, low-power detection |
| Tank and silo level monitoring | Converts echo time to a level in cm, flags empty or no-echo conditions |
| Battery-powered sensor nodes | Removes the need to wake a microcontroller for timing |

**Why dedicated hardware helps.** Deterministic timing, low power, small area, and the block can be reused in any sensor node.

---

## 3. Architecture and working principle

### Block diagram

```
                 +------------------------------------------------+
   clk --------->|                                                |
   rst_n ------->|                                                |--> trig
   en ---------->|           ultrasonic_ranger                    |--> distance_cm[8:0]
   thr[7:0] ---->|                                                |--> valid
                 |  +---------+   +-------+   +--------------+    |--> err
   echo -------->|->| 2-FF    |-->|  FSM  |-->| timer (20b)  |    |--> alarm
   (async)       |  | sync    |   |6 state|   | sub-ctr (12b)|    |
                 |  +---------+   +-------+   | cm-ctr  (9b) |    |
                 |                            +--------------+    |
                 +------------------------------------------------+
```

### Measurement sequence

1. After reset, when `en` is high, wait the inter-measurement gap (60 ms).
2. Drive `trig` high for 10 µs to start the sensor.
3. Wait for `echo` to go high. If it does not within 30 ms, report a no-echo error.
4. While `echo` is high, count clock cycles. Every 58 µs worth of cycles, add 1 to the centimetre counter.
5. When `echo` falls, copy the count to `distance_cm`, pulse `valid` for one clock, and set `alarm` if the distance is less than `thr`.
6. If the count reaches 400 cm, set `err`, then wait for `echo` to fall (the DRAIN state) before starting the next measurement.
7. Repeat from step 1.

### FSM states

| Code | State | What happens |
|---|---|---|
| 0 | `S_IDLE` | Outputs idle, waiting for `en` |
| 1 | `S_GAP` | Waits the gap between measurements |
| 2 | `S_TRIG` | Holds `trig` high for `TRIG_CYCLES` clocks |
| 3 | `S_WAIT` | Waits for the echo to start, with timeout |
| 4 | `S_MEAS` | Counts the echo width in centimetres |
| 5 | `S_DRAIN` | After an out-of-range echo, waits for `echo` to fall |

### Sequential elements and where the 59 flip-flops come from

| Register | Width | Purpose |
|---|---|---|
| `echo_sync` | 2 | Two-stage synchronizer for the asynchronous `echo` input |
| `state` | 3 | FSM state |
| `timer` | 20 | Gap, trigger width and timeout counting |
| `sub` | 12 | Clock cycles inside the current centimetre |
| `cm` | 9 | Running centimetre count |
| `distance_cm` | 9 | Output register holding the last result |
| `trig`, `valid`, `err`, `alarm` | 1 each | Output registers |
| **Total** | **59** | Matches the 59 `dfxtp_1` cells in the synthesis report |

### Key design decisions

- **Divider-free conversion.** A sub-counter counts `CLKS_PER_CM` clock cycles (580 at 10 MHz = 58 µs) and then increments the centimetre counter, so there is no divider. The result is the whole number of centimetres (the fraction is truncated).
- **2-flip-flop synchronizer** on `echo`, because the sensor output is asynchronous to the clock and could otherwise cause metastability.
- **Synchronous active-low reset**, which keeps timing analysis simple.
- **DRAIN state**, so a very long echo can't be mistaken for the start of the next measurement.
- **Errors never freeze the FSM.** Both error cases return to the normal gap-and-measure cycle.

---

## 4. Interface (inputs and outputs)

| Signal | Dir | Width | Description |
|---|---|---|---|
| `clk` | in | 1 | System clock, 10 MHz |
| `rst_n` | in | 1 | Synchronous active-low reset |
| `en` | in | 1 | 1 = run measurements, 0 = idle |
| `echo` | in | 1 | Asynchronous echo pulse from the sensor |
| `thr` | in | 8 | Alarm threshold in cm |
| `trig` | out | 1 | Trigger pulse to the sensor |
| `distance_cm` | out | 9 | Last measured distance in cm |
| `valid` | out | 1 | One-cycle pulse when a new result is ready |
| `err` | out | 1 | No echo, or out of range |
| `alarm` | out | 1 | Object closer than `thr` |

Behaviour notes:
- Valid range is 0 to 399 cm. A reading of 400 cm or more sets `err` and reports `distance_cm` = 400.
- A no-echo timeout sets `err` and reports `distance_cm` = 0.
- `alarm` uses a strict `<` comparison, so a distance equal to the threshold does not alarm.
- `err` stays high until the next valid measurement completes.

---

## 5. Design parameters and the scaled simulation

| Parameter | Default (10 MHz) | Meaning |
|---|---|---|
| `TRIG_CYCLES` | 100 | 10 µs trigger pulse |
| `CLKS_PER_CM` | 580 | 58 µs per centimetre |
| `GAP_CYCLES` | 600000 | 60 ms between measurements |
| `WAIT_CYCLES` | 300000 | 30 ms maximum wait for echo start |
| `MAX_CM` | 400 | Out-of-range limit |

Constraint: `GAP_CYCLES`, `WAIT_CYCLES` and `TRIG_CYCLES` must be below 2^20.

With the real values one measurement takes tens of milliseconds, which means millions of clock cycles per test. So the testbench (and the gate-level run) use **scaled-down values**. The clock stays at 100 ns, only the counts shrink. **Use this table to convert the time axis of every waveform below back to real hardware:**

| Quantity | Simulation (scaled) | Real hardware (10 MHz) |
|---|---|---|
| 1 cm of echo | 10 clocks = **1 µs** | 580 clocks = 58 µs |
| Trigger pulse | 10 clocks = **1 µs** | 100 clocks = 10 µs |
| Gap between measurements | 50 clocks = **5 µs** | 600000 clocks = 60 ms |
| No-echo timeout | 800 clocks = **80 µs** | 300000 clocks = 30 ms |
| Sensor model reaction delay | 20 clocks = **2 µs** | about 0.5 ms (sensor dependent) |
| 400 cm out-of-range echo | 4000 clocks = **400 µs** | 23.2 ms |

Because 1 cm equals 1 µs in the simulation, an echo pulse of N µs on the waveform means N cm.

---

## 6. Verification plan

The testbench (`tb_ultrasonic_ranger.v`) is self-checking and prints PASS or FAIL per check. A behavioural HC-SR04 model answers each trigger with an echo of a chosen length (`d × 10 + 5` clock cycles for a distance of `d` cm; the extra 5 cycles keep the pulse safely inside the centimetre window instead of exactly on a boundary).

| # | Test | Expected result |
|---|---|---|
| 1 | Reset asserted | All outputs low |
| 2 | `en` = 0 | No trigger pulse |
| 3 | Trigger width | `trig` high for exactly `TRIG_CYCLES` clocks |
| 4 | Normal distances (50, 150 cm) | Correct distance, `err` = 0 |
| 5 | Alarm boundary (99 and 100 cm, threshold 100) | Alarm at 99, none at 100 |
| 6 | Very close (0 and 1 cm) | Correct distance, alarm set |
| 7 | Maximum valid range (399 cm) | Correct distance, `err` = 0 |
| 8 | Threshold changes (0, 200, 255) | Alarm follows the threshold |
| 9 | Out of range (long echo) | `err` = 1, distance = 400, alarm = 0 |
| 10 | No echo from the sensor | Timeout, `err` = 1, distance = 0 |
| 11 | Recovery after each error | Next measurement is correct |
| 12 | `en` toggled off and on | Triggering stops, then resumes |
| 13 | Reset in the middle of a measurement | Outputs cleared, normal operation afterwards |
| 14 | Spacing between measurements | At least `GAP_CYCLES` between triggers |

Result: **50 checks passed, 0 failed**, both on the RTL and on the synthesized gate-level netlist.

---

## 7. Project status

| Stage | Status |
|---|---|
| Application definition and proposal | Done |
| RTL design (synthesizable Verilog) | Done |
| Self-checking testbench | Done |
| RTL simulation and waveform review | Done |
| Synthesis in Yosys (sky130, typical corner) | Done |
| Pre-synthesis vs post-synthesis comparison | Done |
| Gate-level simulation | Done |
| Static timing analysis at **all corners** | Done |
| Floorplan, placement, clock tree, routing | Done |
| GDS generation | Done |

Static timing analysis, placement, routing and GDS need OpenLane (or OpenROAD and OpenSTA with the slow and fast corner libraries). They are not installed on the first VM, which is why they are listed as remaining work.

---

## 8. Repository structure

```
ultrasonic_ranger/
|-- ultrasonic_ranger.v            RTL design
|-- tb_ultrasonic_ranger.v         Self-checking testbench (RTL)
|-- tb_gls.v                       Testbench copy for gate-level simulation (no parameter override)
|-- ultrasonic_ranger_net.v        Synthesized netlist, test parameters (used for GLS)
|-- ultrasonic_ranger_final_net.v  Synthesized netlist, real parameters
|-- synth_final.ys                 Yosys script for the final synthesis
|-- synth_final.log                Yosys log
|-- synth_final_stat.txt           Cell count and area report
|-- rtl.vcd, gls.vcd               RTL and gate-level waveforms
|-- config.json                    OpenLane configuration (for the physical design stage)
|-- run_sim.sh                     Helper script for RTL simulation
|-- project_proposal.md            Proposal and checklist
`-- images/                        Screenshots used in this README
```

---

## 9. Execution on the virtual machine, with results explained

Environment: VirtualBox VM (Ubuntu-based, VSD workshop image) with Icarus Verilog, Yosys, GTKWave and the sky130 typical-corner library (`sky130_fd_sc_hd__tt_025C_1v80.lib`). Every step below lists the commands, what each one does, what the screenshot shows, and **why the design produced that result**.

### 9.1 Project folder setup

```bash
cd ~/Desktop
mkdir -p ultrasonic_proj
cp ultrasonic_ranger.v tb_ultrasonic_ranger.v ultrasonic_proj/
cd ultrasonic_proj
ls
```

| Command | Purpose |
|---|---|
| `mkdir -p` | Create the project folder (no error if it exists) |
| `cp` | Copy the RTL and testbench into it |
| `ls` | Confirm the files are there |



**Why the waveform file is so large.** The `.vcd` file records every change of every signal for the whole 2.5 ms simulation (about 25,000 clock cycles), including all internal registers of the design, so it is by far the biggest file. That is expected, and it is also why the file is regenerated each run instead of being edited.

---

### 9.2 RTL simulation

```bash
iverilog -g2005 -o rtl_sim ultrasonic_ranger.v tb_ultrasonic_ranger.v
vvp rtl_sim
```

| Command | Purpose |
|---|---|
| `iverilog -g2005 -o rtl_sim ...` | Compile the RTL and testbench into a simulation program (Verilog-2005 mode) |
| `vvp rtl_sim` | Run the simulation. The testbench prints each measurement and a final PASS/FAIL count, and writes `tb_ultrasonic_ranger.vcd` |

Expected ending: `PASS checks: 50   FAIL checks: 0` and `*** ALL TESTS PASSED ***`.

**Why this result.** The testbench compares every output (`distance_cm`, `err`, `alarm`) against the expected value after each measurement, and it also checks trigger width, reset behaviour and the gap between triggers. All 50 comparisons agreed with the design, so the RTL implements the specification, including the boundary and error cases.

---

### 9.3 RTL waveform review in GTKWave

```bash
mv tb_ultrasonic_ranger.vcd rtl.vcd
gtkwave rtl.vcd &
```

| Command | Purpose |
|---|---|
| `mv ... rtl.vcd` | Rename the waveform so the later gate-level run does not overwrite it |
| `gtkwave rtl.vcd &` | Open the waveform viewer in the background |

Signals added in this order: `clk`, `rst_n`, `en`, `thr`, `echo`, `trig`, `state`, `distance_cm`, `valid`, `err`, `alarm` (`thr`, `state` and `distance_cm` shown in decimal). The time window is set with the From/To boxes at the top.

#### Screenshot 0: the unzoomed first view (a common beginner problem)







<img width="1680" height="893" alt="gtkwave1" src="https://github.com/user-attachments/assets/ed0546db-02e3-4675-93c1-c7338b90d8ba" />


**What you see.** The ruler reads 0 to 2 ps, most signals are red, and `state` and `distance_cm` show `xxx`.

**Why.** GTKWave opens zoomed in to the very beginning of the simulation, before the first clock edge. Registers have no value until the first rising clock edge in which reset is applied, and the design uses a **synchronous** reset, so until that edge every register is "unknown" (red). The values `thr = 100` already show because the testbench drives it directly. This is not a design bug. The fix is *Zoom Fit* and typing a proper From/To window.

#### Screenshot 1: reset and startup (0 to 3 µs)






<img width="1680" height="893" alt="gtkwave1" src="https://github.com/user-attachments/assets/95346b97-a49a-4a24-ae4c-e804161221e8" />


**What you see.**
- `rst_n` is low at the start and rises at about 0.45 µs.
- `en` stays low and rises at about 2.5 µs.
- `state` is 0, then changes to **1** just after `en` rises.
- `trig`, `valid`, `err`, `alarm` and `distance_cm` are all 0 once reset has been applied.
- Red values appear only in the first clock period.

**Why.** The testbench holds `rst_n` low for 5 clock periods and then releases it, waits another 20 clocks, and only then raises `en`. While reset is active, the synchronous reset forces every register to 0 on each clock edge, so the outputs are clean. In IDLE (state 0) the FSM ignores everything except `en`; on the first clock edge after `en` goes high it moves to GAP (state 1) to start counting the gap before the first measurement. The small red area at time 0 is the "unknown before the first clock edge" behaviour explained above.

**What it proves.** Reset puts the design into a known state, outputs are safe after reset, and the circuit does nothing until enabled.

#### Screenshot 2: first four measurements (early overview)







<img width="1680" height="893" alt="gtk2" src="https://github.com/user-attachments/assets/f66d7385-cf55-45cf-bf75-3dbaf4286544" />


**What you see.** Echo pulses of increasing length, `state` sitting at 4 while each echo is high, and `distance_cm` stepping **0, 50, 150, 100**, with `alarm` high only during the 50 cm result.

**Why.** Each time `echo` falls, the FSM copies the centimetre count to `distance_cm` and pulses `valid`. The pulse widths in the picture (about 50, 150 and 100 µs) equal the distances because 1 cm = 1 µs in the scaled simulation. The 50 cm result is below the threshold of 100, so `alarm` is high. For 150 cm the alarm drops. For 100 cm it stays low because the comparison is strict `<`, and 100 is not less than 100.

#### Screenshot 3: one complete measurement (0 to 70 µs)







<img width="1680" height="893" alt="gtk2" src="https://github.com/user-attachments/assets/cd4d2b6d-b02d-4a2f-989f-0665cd8b09d7" />


**What you see, in order.**
1. `en` rises at about 2.5 µs and `state` goes 0 → **1** (GAP).
2. After the 5 µs gap, `state` goes to **2** and `trig` is high for about 1 µs.
3. `state` goes to **3** (WAIT) as `trig` falls.
4. About 10 µs: `echo` rises, `state` goes to **4** (MEAS) and stays there.
5. About 60.5 µs: `echo` falls, `valid` pulses for one clock, `distance_cm` changes from 0 to **50**, `alarm` rises, and `state` returns to **1**.
6. After another 5 µs gap, `trig` fires again at about 66 µs and `state` runs 2 → 3 → 4 for the next measurement.

**Why these numbers.**
- The gap is 5 µs because the scaled `GAP_CYCLES` is 50 clocks of 100 ns. The trigger is 1 µs because `TRIG_CYCLES` is 10 clocks.
- The testbench's sensor model starts the echo 20 clocks (2 µs) after the trigger rising edge, which is why `echo` rises shortly after `trig` ends.
- The echo is 505 clocks long (50 cm × 10 clocks + 5 extra). The FSM counts the clocks during which the echo is high (504 of them, because the first high cycle is spent leaving WAIT), which is 50 full groups of 10 clocks, so the count is exactly **50 cm**. The leftover 4 clocks do not make a full centimetre and are truncated.
- `alarm` = 1 because 50 < `thr` = 100.
- `valid` is a single-clock pulse by design, so downstream logic can capture the result exactly once.

**What it proves.** The FSM visits its states in the correct order (0 → 1 → 2 → 3 → 4 → 1), the trigger width is right, the echo width is converted correctly, and the result/alarm logic works.

#### Screenshot 4: alarm boundary







<img width="1680" height="893" alt="gtk3" src="https://github.com/user-attachments/assets/1802fc0e-55c6-419a-b90d-34f4983e48d0" />


**What you see.** `distance_cm` steps 50, **150**, **100**, **99**, 1, 0. `alarm` is high for 50, low for 150, low for 100, high again for 99, 1 and 0. (At this zoom a short value such as 99 may be cut off, but its alarm transition is visible.)

**Why.** The alarm is registered as `cm < thr` with `thr` = 100:
- 150 is not less than 100, so no alarm.
- 100 is not less than 100, so still no alarm. This is the **equality boundary**, and the design correctly treats it as "not closer than the threshold".
- 99 is less than 100, so the alarm turns on.
- 1 and 0 are far below the threshold, so the alarm stays on.

**What it proves.** The comparator is strict, there is no off-by-one error at the threshold, and very small distances (including 0) are handled.

#### Screenshot 5: threshold changes and out-of-range (about 1300 to 1760 µs)







<img width="1680" height="893" alt="gtk4" src="https://github.com/user-attachments/assets/68060fd6-d226-40b1-bfd5-8283f572fa76" />


**What you see, in order.**
- `thr` changes 100 → 200 → 0 → 255 → 100 earlier in the run, and the alarm follows: 150 cm with threshold 200 gives an alarm, 10 cm with threshold 0 gives no alarm, 254 cm with threshold 255 gives an alarm, and the 399 cm reading with threshold 100 gives no alarm.
- Near 1300 µs, `echo` goes high and **stays high for about 420 µs**.
- At about 1700 µs `state` jumps to **5** (DRAIN), `err` goes high, `distance_cm` shows **400** and `alarm` is 0.
- `echo` falls at about 1720 µs, `state` returns to 1, and then the next measurement runs.
- At about 1758 µs the next result arrives: `distance_cm` = **30**, `err` back to 0, `alarm` = 1.

**Why.**
- The alarm follows the threshold because it is just `cm < thr` evaluated at the end of every measurement. With `thr` = 0 nothing can be below it (so no alarm for 10 cm), and with `thr` = 255 almost everything below 255 triggers it (254 does).
- The out-of-range echo models a sensor that sees nothing: the echo stays high for 420 µs (a distance equivalent of 420 cm). The centimetre counter reaches its limit after 400 ticks (400 µs), so the design stops counting, reports 400 with `err` = 1, and clears the alarm because a distance of "400 or more" is not a real nearby object.
- **DRAIN (state 5)** waits until the echo actually falls (about 20 µs later). Without it, the FSM would start a new cycle while the sensor was still signalling and misread the end of the old echo as a new one.
- `err` stays high until the next good measurement completes (30 cm), which shows the circuit **recovers** from the fault by itself.

**What it proves.** Threshold handling works across the full 8-bit range, out-of-range echoes are detected and flagged, and the FSM resumes normal operation afterwards.

#### Screenshot 6: no-echo timeout (about 1760 to 1925 µs)







<img width="1680" height="893" alt="gtk5" src="https://github.com/user-attachments/assets/fb7d3dab-ed8c-4636-a332-09d592fe2563" />


**What you see.**
- After the 30 cm measurement, `trig` pulses once at about 1763 µs, but **`echo` stays low**.
- `state` stays at **3** (WAIT) for about 80 µs.
- At about 1843 µs `valid` pulses, `distance_cm` becomes **0**, `err` goes high and `alarm` goes low.
- The next trigger gets an echo, and about 1921 µs later `distance_cm` = **70** with `err` back to 0 and `alarm` = 1.
- Right after that, `en` falls and `state` returns to **0** (IDLE).

**Why.**
- The testbench sets the sensor model to "do not answer" for this measurement, simulating an unplugged or blocked sensor.
- The FSM waits in WAIT up to `WAIT_CYCLES` = 800 clocks = 80 µs (scaled). When the timeout expires with no echo, it reports distance 0 with `err` = 1 and `alarm` = 0 (no valid object), pulses `valid`, and goes back to the gap state instead of waiting forever.
- The 70 cm result shows the recovery works.
- When the testbench drops `en`, the FSM sees `en` = 0 in GAP and moves to IDLE, which is why `state` returns to 0 and no further triggers appear. This is the "enable stops triggering" test.

**What it proves.** A missing sensor can't hang the circuit, the error is reported distinctly from an out-of-range reading (distance 0 versus 400), and `en` pauses the whole operation cleanly.

---

### 9.4 Pre-synthesis statistics (generic RTL cells)

Started `yosys` and typed at the `yosys>` prompt:

```
read_verilog ultrasonic_ranger.v
chparam -set TRIG_CYCLES 10 -set CLKS_PER_CM 10 -set GAP_CYCLES 50 -set WAIT_CYCLES 800 -set MAX_CM 400 ultrasonic_ranger
hierarchy -check -top ultrasonic_ranger
proc
flatten
opt
stat
exit
```

| Command | Purpose |
|---|---|
| `read_verilog` | Load the RTL |
| `chparam -set ...` | Override the parameters with the small test values, so the netlist matches the testbench |
| `hierarchy -check -top` | Resolve the design and set the top module |
| `proc` | Convert always blocks into registers and logic |
| `flatten`, `opt` | Flatten the hierarchy and do generic optimization |
| `stat` | Print the cell statistics |






<img width="1680" height="893" alt="presynth1" src="https://github.com/user-attachments/assets/5ec49767-055a-4b38-9cd4-de4c775fba5a" />


**What the screenshot shows.** Yosys reads the file, then `chparam` prints the five parameter values (10, 10, 50, 800, 400) twice, once when setting them and once when it regenerates the design with them. `hierarchy` reports `ultrasonic_ranger` as the top module with no unused modules, and `proc` runs its sub-passes (it notes marking 11 switch rules as full-case, which is the FSM's case statement).

**Why.** `chparam` is needed because the testbench overrides the parameters, and the later gate-level netlist must contain the same values. Only one module exists, so the hierarchy is trivial.






<img width="1680" height="893" alt="presynth2" src="https://github.com/user-attachments/assets/f50b8706-b59d-407c-b817-3d4bade3993f" />


**What the screenshot shows.** 106 cells, 25 port bits on 10 ports, 0 memories and 0 processes, made of generic cells: `$add` ×3, `$eq` ×10, `$ne` ×12, `$lt` ×1, `$mux` ×49, `$pmux` ×9, `$reduce_and` ×6, `$reduce_bool` ×3, `$logic_not` ×2, `$logic_or` ×1, and flip-flop cells `$sdff` ×2 and `$sdffe` ×8.

**Why each group appears.**
- `$add` ×3: the three incrementers (`timer + 1`, `sub + 1`, `cm + 1`).
- `$lt` ×1: the single comparator `cm < thr` for the alarm.
- `$eq` / `$ne` ×22: the state comparisons and the terminal-count checks (`timer == GAP_CYCLES-1`, `sub == CLKS_PER_CM-1`, and so on).
- `$mux` / `$pmux` ×58: the next-state and next-value selection inside the FSM case statement.
- `$sdff` / `$sdffe` ×10: Yosys's multi-bit flip-flop cells with **synchronous reset**, with or without an enable. These 10 word-wide cells together hold the 59 register bits listed in section 3.
- 0 memories, no multipliers, no dividers: confirms the divider-free design goal.

The count of 106 is small because each cell is still a word-wide operation, not yet broken into gates.

---

### 9.5 Synthesis in Yosys (sky130 mapping)

Library path used: `/home/vsduser/sky130RTLDesignAndSynthesisWorkshop/lib/sky130_fd_sc_hd__tt_025C_1v80.lib`

```
read_liberty -lib <path to .lib>
read_verilog ultrasonic_ranger.v
chparam -set TRIG_CYCLES 10 -set CLKS_PER_CM 10 -set GAP_CYCLES 50 -set WAIT_CYCLES 800 -set MAX_CM 400 ultrasonic_ranger
hierarchy -check -top ultrasonic_ranger
proc
flatten
opt
fsm
opt
memory
opt
techmap
opt
dfflibmap -liberty <path to .lib>
abc -liberty <path to .lib>
opt_clean -purge
check
stat -liberty <path to .lib>
write_verilog -noattr ultrasonic_ranger_net.v
exit
```

| Command | Purpose |
|---|---|
| `read_liberty -lib` | Load the standard-cell library (cell definitions only) |
| `fsm` | Detect and optimize the finite state machine |
| `memory` | Handle memories (none in this design) |
| `techmap` | Map generic cells to simple gates |
| `dfflibmap` | Map flip-flops to library flip-flops |
| `abc -liberty` | Optimize and map the combinational logic to library gates |
| `opt_clean -purge` | Remove unused cells and wires |
| `check` | Check for undriven or multiply driven signals |
| `stat -liberty` | Report cell counts and chip area |
| `write_verilog -noattr` | Write the gate-level netlist |







<img width="1680" height="893" alt="stat result1" src="https://github.com/user-attachments/assets/e6c204bb-483d-4a2d-8f25-4812eca6edd5" />


**What the screenshot shows (statistics, test parameters).** 355 cells in sky130 library gates. The biggest groups are `nor2_1` ×45, `a21oi_1` ×40, `nand2_1` ×33, `a22oi_1` ×23, `o21ai_0` ×23, `a31oi_1` ×12, `nor3_1` ×14 and `clkinv_1` ×13. There are exactly **59 `dfxtp_1`** flip-flops.






<img width="1680" height="893" alt="stat resullt and write command dump 2" src="https://github.com/user-attachments/assets/a7dcad59-d913-4fb0-a63c-da697eeb499f" />


**What the screenshot shows (area and netlist).** Chip area is **2830.21 µm²**, of which 1181.13 µm² (41.73%) is sequential (flip-flops). The command `write_verilog` then dumps module `ultrasonic_ranger` into `ultrasonic_ranger_net.v`.

**Why these numbers.**
- **59 flip-flops** equals the sum of the register widths listed in section 3 (2 + 3 + 20 + 12 + 9 + 9 + 4). It matches because `dfflibmap` simply maps every register bit to one `dfxtp_1` cell.
- **1181.13 µm²** = 59 × 20.02 µm² (the area of one `dfxtp_1`), so the sequential share is exactly the flip-flop count times the cell area.
- **355 cells versus 106 before**: each word-wide generic cell is broken into single-bit gates. For example, the 20-bit timer incrementer and its comparators turn into many small gates, and the 49 muxes become and-or-invert gates.
- **NOR/NAND/AOI dominate** because sky130's inverting gates are the smallest and fastest CMOS gates, and ABC picks them to minimize area.
- The `clkinv_1` cells are ordinary inverters (sky130's inverter cell used by the mapper).

---

### 9.6 Gate-level simulation (post-synthesis)

```bash
perl -0pe 's/#\(\.TRIG_CYCLES.*?\.MAX_CM\(MAX_CM\)\)//s' tb_ultrasonic_ranger.v > tb_gls.v
M=~/sky130RTLDesignAndSynthesisWorkshop/my_lib/verilog_model
iverilog -g2005 -DFUNCTIONAL -DUNIT_DELAY=#1 -o gls_sim tb_gls.v ultrasonic_ranger_net.v $M/primitives.v $M/sky130_fd_sc_hd.v
vvp gls_sim
mv tb_ultrasonic_ranger.vcd gls.vcd
```

| Command | Purpose |
|---|---|
| `perl -0pe ...` | Make a copy of the testbench without the `#(...)` parameter override, because the synthesized netlist has the parameters built in |
| `M=...` | Shortcut to the folder holding the sky130 cell simulation models |
| `iverilog -DFUNCTIONAL -DUNIT_DELAY=#1 ...` | Compile the testbench, the netlist and the cell models (functional models with a 1 time-unit delay per cell) |
| `vvp gls_sim` | Run the gate-level simulation |
| `mv ... gls.vcd` | Keep the gate-level waveform separately from the RTL one |







<img width="1680" height="893" alt="picafterpreanndpostsynth" src="https://github.com/user-attachments/assets/256cdd79-32ca-41d8-ac4f-ff1a00e21ff1" />


**What the screenshot shows.** The top of the screen shows the end of the pre-synthesis `stat` output and the three setup commands (`perl`, `M=...`, `iverilog`), which ran without errors. The `vvp` output then lists every measurement the testbench performed, with the result read from the **gate-level netlist**:

| Test | `distance_cm` | `err` | `alarm` |
|---|---|---|---|
| 50 cm, thr 100 | 50 | 0 | 1 |
| 150 cm | 150 | 0 | 0 |
| 100 cm (equal to thr) | 100 | 0 | 0 |
| 99 cm | 99 | 0 | 1 |
| 1 cm | 1 | 0 | 1 |
| 0 cm | 0 | 0 | 1 |
| 399 cm | 399 | 0 | 0 |
| 150 cm, thr 200 | 150 | 0 | 1 |
| thr = 0, 10 cm | 10 | 0 | 0 |
| 254 cm, thr 255 | 254 | 0 | 1 |
| out of range | 400 | 1 | 0 |
| recovery after error | 30 | 0 | 1 |
| no-echo timeout | 0 | 1 | 0 |
| recovery after timeout | 70 | 0 | 1 |
| resume after enable | 40 | 0 | 1 |
| after mid-run reset | 20 | 0 | 1 |

The summary reads **`PASS checks: 50   FAIL checks: 0`** and **`*** ALL TESTS PASSED ***`**.

**Why this result.** Synthesis is supposed to preserve behaviour: the netlist is the same logic expressed in real library gates. Running the identical self-checking testbench on the netlist and getting identical results is the evidence that Yosys and ABC did not change the function. The simulation also ends at 2,488,851,000 ps versus 2,487,250,000 ps for the RTL run, about 1.6 µs (0.06%) later. That tiny difference comes from the 1-unit delay modelled in each gate and flip-flop, which makes each result appear a few nanoseconds later than in the zero-delay RTL run. It does not change any value.

---

### 9.7 Pre-synthesis vs post-synthesis waveform comparison

```bash
gtkwave rtl.vcd &
gtkwave gls.vcd &
```

The same 10 signals (`clk`, `rst_n`, `en`, `thr`, `echo`, `trig`, `distance_cm`, `valid`, `err`, `alarm`) were added in both windows with identical time windows. `state` was left out because Yosys may recode the FSM and flattening removes the internal names (in the gate-level hierarchy, `dut` contains numbered gate instances such as `_296_`, `_297_`). In each screenshot below, the **top window is the gate-level run (`gls.vcd`)** and the **bottom window is the RTL run (`rtl.vcd`)**.

#### Comparison 1: startup (0 to 3 µs)








<img width="1680" height="893" alt="cmppreandpostsyth1" src="https://github.com/user-attachments/assets/21f2c7b3-060e-4b63-9cdf-0ff77833c52a" />








<img width="1680" height="893" alt="cmppreandpostsynth2" src="https://github.com/user-attachments/assets/82657c89-9a5a-45e7-ba3b-6a302531ef96" />


**What you see.** Both windows show `rst_n` releasing at about 0.45 µs and `en` rising at about 2.5 µs, with a short red/unknown period at the very start. `distance_cm` shows `X` briefly and then `0` in both.

**Why.** Both versions use the same synchronous reset, so nothing is defined until the first clock edge. The gate-level version's registers are the library's `dfxtp` flip-flops, which also start unknown, so the startup looks the same.

#### Comparison 2: first measurement (0 to 70 µs)







<img width="1680" height="893" alt="cmppreandpostsynth3" src="https://github.com/user-attachments/assets/94567f96-ec77-40fb-a7d0-052a0e8021a8" />


**What you see.** In both windows: `trig` pulses at about 8 µs, `echo` rises at about 10 µs and falls at about 60.5 µs, `valid` pulses at that moment, `distance_cm` becomes 50, `alarm` rises, and the second `trig` appears at about 66 µs.

**Why identical.** The gate-level logic is functionally the same as the RTL, and a 1-unit gate delay (about 1 ns) is invisible against a 70 µs window.

#### Comparison 3: alarm boundary (0 to 460 µs)










<img width="1680" height="893" alt="cmppreandpostsynth4" src="https://github.com/user-attachments/assets/e560e48c-b1f1-464a-b70c-251cdbdaf6eb" />


**What you see.** Both windows show the same sequence 50, 150, 100, 99, 1, 0 with `valid` pulses at the end of each echo, `err` flat at 0, and `alarm` high for 50, low for 150 and 100, then high from 99 onwards.

**Why identical.** The `<` comparator was mapped into gates, but a logically equivalent comparator gives the same result for every input, including the equality case (100) which is the most error-prone.

#### Comparison 4: whole-run overview with threshold changes and out-of-range









<img width="1680" height="893" alt="cmppreandpostsynth5" src="https://github.com/user-attachments/assets/f0047eb3-1cbc-40a4-b2a3-94da204cbefc" />


**What you see.** The full run from 0 to about 1800 µs in both windows. `thr` changes 100 → 200 → 0 → 255 → 100. `distance_cm` steps through 0, 50, 150, 100, 0 and 1 (squeezed), 399, 150 (shown as `+`), 10, 254, 400 and 30. `alarm` and `err` match between windows, and the `err` pulse for the out-of-range case appears near the end.

**Note.** The From box says 1330 µs, but GTKWave kept the full 0-to-1800 µs view. That does not matter here, because the overview is more informative: it lets the reader compare the whole test sequence at once. The `+` is how GTKWave labels a value too wide for the space available, not a design value.

**Why identical.** Same reason as above: all outputs come from equivalent logic, so every threshold case and the error case match step for step.

#### Comparison 5: no-echo timeout and recovery (about 1705 to 1950 µs)








<img width="1680" height="893" alt="cmppreandpostsynth6" src="https://github.com/user-attachments/assets/e5f523a6-ecf9-4c8b-9c33-c77512adbefe" />


**What you see.** Both windows show 30 cm changing to 0 at about 1843 µs with `err` and `valid`, the 70 cm recovery at about 1921 µs, and `en` falling right after. The timeout length matches in the two windows.

**Note.** The window starts a little earlier than the 1800 µs typed in the From box, because GTKWave adjusts the view to fit.

**Why identical.** The timeout counter (`timer == WAIT_CYCLES-1`) and error logic were synthesized into gates with the same function, so the 80 µs wait and the error flags are reproduced exactly.

**Overall conclusion.** Values and timing of all outputs match between RTL and gate level (edges differ by about 1 ns from the unit-delay cell models, which is not visible at these scales), so synthesis preserved the design function.

---

### 9.8 Final synthesis with the real parameters

A script was used so all commands run in one go from the normal terminal (not inside the `yosys>` prompt):

```bash
LIB=/home/vsduser/sky130RTLDesignAndSynthesisWorkshop/lib/sky130_fd_sc_hd__tt_025C_1v80.lib
cat > synth_final.ys << EOF
read_liberty -lib $LIB
read_verilog ultrasonic_ranger.v
hierarchy -check -top ultrasonic_ranger
proc; flatten; opt; fsm; opt; memory; opt
techmap; opt
dfflibmap -liberty $LIB
abc -liberty $LIB
opt_clean -purge
check
tee -o synth_final_stat.txt stat -liberty $LIB
write_verilog -noattr ultrasonic_ranger_final_net.v
EOF
yosys -l synth_final.log -s synth_final.ys
cat synth_final_stat.txt | tail -20
```

| Command | Purpose |
|---|---|
| `LIB=...` | Store the library path in a shell variable |
| `cat > synth_final.ys << EOF ... EOF` | Write the Yosys script to a file |
| `yosys -l synth_final.log -s synth_final.ys` | Run the script and save a log |
| `tee -o ... stat` | Save the statistics report to a text file |
| `cat ... \| tail -20` | Show the end of the report, including chip area |

Note that the script has no `chparam` line, so the synthesis uses the **real 10 MHz parameters**.










<img width="1680" height="893" alt="finalsythstat1" src="https://github.com/user-attachments/assets/00767b08-b358-4f29-924c-15556d42a30c" />

**What the screenshot shows.** The terminal echoes the script being written, Yosys 0.47 starts, and its first step reports `Imported 418 cell types from liberty file` before reading the Verilog and running the hierarchy and `proc` passes. **Why:** the liberty file lists 418 cells of the sky130 library, which is the pool ABC can choose from.








<img width="1680" height="893" alt="finalsynthstat2" src="https://github.com/user-attachments/assets/8276e361-4b85-4b69-a38d-3399f5520e6a" />


**What the screenshot shows.** The `check` pass reports **"Found and reported 0 problems"**, followed by the statistics: 357 cells, 25 port bits, 0 memories, 0 processes.

**Why.** `check` looks for undriven wires, multiply driven nets and combinational loops. Zero problems means the netlist is structurally clean.







<img width="1680" height="893" alt="finalsynthstat3" src="https://github.com/user-attachments/assets/45a628e8-bfb9-4c01-b225-c01286309a9d" />



**What the screenshot shows.** The cell list (for example `a21oi_1` ×45, `nor2_1` ×48, `nand2_1` ×29, `a22oi_1` ×27, `clkinv_1` ×18, `and2_0` ×12, `o21ai_0` ×17, `dfxtp_1` ×59) and **chip area 2840.224 µm²**, of which 1181.1328 µm² (41.59%) is sequential.








<img width="1680" height="893" alt="finalsynthstat4" src="https://github.com/user-attachments/assets/34351763-c20d-4c8b-8745-e69683ff0422" />


**What the screenshot shows.** The same area figure at the end of the run, the Verilog backend dumping module `ultrasonic_ranger`, and the summary line **"Warnings: 24 unique messages, 24 total"**.

**Why the area is slightly larger than the test-parameter run.** The real design compares against much larger constants (580, 600000, 300000 instead of 10, 50, 800), so the comparators for `timer`, `sub` and `cm` need a few more gates. Total area rises only from 2830.21 µm² to 2840.22 µm² (about 0.35%), the flip-flop part is exactly the same (59 flip-flops, 1181.13 µm²), and the combinational part grows by about 10 µm². This also confirms the test-parameter netlist was a fair stand-in for the real design.

**About the 24 warnings.**
- 22 or more are `Inference failed ... sdf.../sedf...` messages. They come from `dfflibmap` reading the library: sky130 contains **scan flip-flops** (`sdfxtp`, `sedfxtp`, and others) with extra scan inputs, which Yosys cannot map to a plain flip-flop, so it skips them and uses the ordinary `dfxtp_1`. That is exactly what the design uses.
- Two are ABC notes: "Detected 9 multi-output gates (for example `fa_1`)", meaning ABC does not use the library's full-adder cells and builds adders from basic gates, and "The network is combinational", which is only a status message.
- None of them mention a signal of this design, and `check` found 0 problems, so they are harmless.

---

### 9.9 Tool check for the next stage

```bash
which sta openroad magic klayout docker
find / -maxdepth 4 -iname "*openlane*" 2>/dev/null
ls ~/sky130RTLDesignAndSynthesisWorkshop/lib/
df -h ~
```

| Command | Purpose |
|---|---|
| `which ...` | Check which timing and physical-design tools are installed |
| `find / ... openlane` | Search for an OpenLane installation |
| `ls .../lib/` | See which corner libraries exist |
| `df -h ~` | Check free disk space |

**Result on the first VM.** `which` printed nothing (no `sta`, `openroad`, `magic`, `klayout` or `docker`), `find` found no OpenLane, `lib/` contained only `sky130_fd_sc_hd__tt_025C_1v80.lib`, and the disk had about 7.7 GB free.

**Why this matters.** Yosys cannot do timing analysis or physical design. Static timing analysis at **all corners** needs the slow (`ss_100C_1v60`) and fast (`ff_n40C_1v95`) libraries as well as an STA engine, and placement and routing need OpenROAD or OpenLane. The first VM has none of them, and about 7.7 GB is too tight for installing OpenLane, so the next stages were moved to a second VM.



## 10. Results summary

| Metric | Value |
|---|---|
| RTL simulation | 50 / 50 checks passed |
| Gate-level simulation | 50 / 50 checks passed |
| Flip-flops | 59 (`dfxtp_1`) |
| Pre-synthesis cells (generic) | 106 |
| Post-synthesis cells, test parameters | 355 |
| Post-synthesis cells, real parameters | 357 |
| Chip area, test parameters | 2830.21 µm² |
| Chip area, real parameters | 2840.22 µm² |
| Sequential share of area | about 41.6% |
| Memories | 0 |
| Dividers / multipliers | none |
| Design `check` problems | 0 |
| Clock | 10 MHz (100 ns) |
| STA at all corners, placement, routing, GDS | Pending |

**Result-to-reason summary.**

| Result | Reason |
|---|---|
| 50 / 50 checks pass | Design matches the specification, including boundaries and errors |
| Alarm off at exactly 100 cm, on at 99 | Comparison is a strict `cm < thr` |
| 399 cm valid, 400 cm flagged `err` | Counter stops when it would count the 400th centimetre |
| No-echo gives distance 0, out-of-range gives 400 | Two separate error paths, so the user can tell them apart |
| 59 flip-flops | 2 + 3 + 20 + 12 + 9 + 9 + 4 register bits |
| Area rises only 0.35% with real parameters | Only comparator constants change, flip-flops are identical |
| RTL and gate-level waveforms match | Synthesis preserves logic, and a 1 ns gate delay is negligible against a µs scale |

---

## 11. Challenges faced and how they were solved

| # | Challenge | Cause | Solution |
|---|---|---|---|
| 1 | Waveform looked empty and red in GTKWave | The view started at 0 to 2 ps, before the first clock edge, and registers are unknown until then (synchronous reset) | Used **Zoom Fit** and typed the From/To times with `ns` and `us` units |
| 2 | GTKWave ignored the From value in some views | The window kept or adjusted the visible range | Used the overview for the whole run and re-entered the times for detailed views |
| 3 | Gate-level simulation could not use the original testbench | The testbench overrides module parameters, but the synthesized netlist has no parameters | Synthesized with `chparam` using the same small values as the testbench and made `tb_gls.v` with the override removed |
| 4 | Real parameters would make gate-level simulation extremely long | 60 ms gaps at 10 MHz mean millions of clock cycles | Used scaled-down parameters for simulation and the real values only for the final area report |
| 5 | Confusion about where to type commands | Commands for the normal terminal and for the `yosys>` prompt look similar | Used a script (`synth_final.ys`) run from the normal terminal for the final synthesis |
| 6 | `state` signal missing in the gate-level waveform | Yosys can recode the FSM and flattening removes internal names | Compared only the top-level ports, which is what matters functionally |
| 7 | 24 Yosys warnings | Scan flip-flop cells in the library, multi-output adders ignored by ABC | Verified they refer only to library cells and that `check` reported 0 problems |
| 8 | Duplicate copies of the source files | Files existed on the Desktop and in the project folder | Compared them with `cmp`, then kept a single working copy inside `ultrasonic_proj` |
| 9 | No STA, placement or routing tools on the first VM | Only Icarus Verilog, Yosys and GTKWave were installed, with only the typical-corner library and about 7.7 GB free | Identified a second VM with OpenSTA sources and Docker, and planned to move only `ultrasonic_ranger.v` there |
| 10 | Copy and paste between the host and the second VM did not work | The shared clipboard needs Guest Additions running inside the guest, not only the VirtualBox setting | Set Shared Clipboard and Drag'n'Drop to Bidirectional (Settings, General, Features), with shared folders as a fallback |
| 11 | Second VM stopped at an `(initramfs)` prompt | The root filesystem was inconsistent (corrupted orphan inode list), probably after an unclean shutdown | See the explanation below the table |
g)







PHYSICAL DESIGN



---

## Table of Contents
1. [Abstract](#1-abstract)
2. [Applications and Problem Statement](#2-applications-and-problem-statement)
3. [Design Overview](#3-design-overview)
4. [Design Features](#4-design-features)
5. [Verification Strategy](#5-verification-strategy)
6. [Execution on the Virtual Machine, Phase 1: Simulation and Synthesis](#6-phase-1-simulation-and-synthesis)
7. [Execution on the Virtual Machine, Phase 2: Environment Setup](#7-phase-2-environment-setup-for-openlane)
8. [Execution on the Virtual Machine, Phase 3: OpenLane Physical Design Flow](#8-phase-3-openlane-physical-design-flow)
9. [Multi-Corner Static Timing Analysis](#9-multi-corner-static-timing-analysis)
10. [Challenges Faced and Solutions](#10-challenges-faced-and-solutions)
11. [Results Summary](#11-results-summary)
12. [Scope, Limitations and Future Work](#12-scope-limitations-and-future-work)
13. [Tools and Environment](#13-tools-and-environment)
14. [Repository Structure](#14-repository-structure)

---

## 1. Abstract

Ultrasonic ranging sensors such as the HC-SR04 report distance as the **width of a pulse**. The sensor emits a burst of 40 kHz sound after receiving a 10 µs trigger pulse, and the ECHO pin stays high for as long as the sound takes to travel to an object and back. Converting that pulse width into a distance normally requires a division (time ÷ 58 µs per centimetre), which is expensive in hardware.

This project implements a complete **ultrasonic distance meter with a programmable proximity alarm** as a synthesizable Verilog module (`ultrasonic_ranger`). The design:

- generates the 10 µs trigger pulse,
- synchronizes the asynchronous echo signal with a 2-flip-flop synchronizer,
- measures the echo width by counting clock cycles, and converts it to **centimetres without a divider** using a "clocks-per-centimetre" tick counter,
- flags an error on no-echo (timeout) and out-of-range conditions, and
- asserts an **alarm** when the measured distance is below a user-programmable threshold.

The design was verified with a self-checking testbench containing a behavioural sensor model (**50 checks, 0 failures**) in both RTL simulation and post-synthesis gate-level simulation (GLS). It was then synthesized to the SKY130 high-density standard cell library (`sky130_fd_sc_hd`) and implemented through the **OpenLane v0.21** flow: floorplan, power distribution network (PDN), global and detailed placement, clock tree synthesis (TritonCTS), global and detailed routing. Sign-off style static timing analysis (OpenSTA) was run **across three PVT corners** (ff_n40C_1v95, tt_025C_1v80, ss_100C_1v60) using the extracted routing parasitics (SPEF).

The routed design meets timing at all three corners with a worst setup slack of **+77.79 ns** and a worst hold slack of **+0.27 ns** at a 10 MHz clock, and finished detailed routing with **zero DRC violations** and **zero antenna violations** at the global routing stage.

---

## 2. Applications and Problem Statement

### 2.1 What the project is used for
- **Parking sensors**: warn the driver when an obstacle is closer than a set distance.
- **Obstacle detection and robot bumpers**: stop or steer a robot before a collision.
- **Tank and bin level monitoring**: measure the distance to a liquid or material surface and flag a "full" or "empty" condition.
- **Proximity and presence detection** in industrial and consumer devices.

### 2.2 What problem it solves
| Problem | How this design addresses it |
|---|---|
| Converting echo time to distance normally needs a hardware divider (large, slow) | Uses a **sub-counter that counts 580 clocks per cm**. Each time it rolls over, a centimetre counter increments, so no division is needed. |
| The echo pin is asynchronous to the system clock, which risks metastability | A **2-flip-flop synchronizer** (`echo_sync`) is placed on the input. |
| Sensors can miss or return nothing | A **wait timeout** (30 ms) raises `err` and returns 0 cm. |
| Distant or no object makes the echo very long | A **range limit** (400 cm) raises `err` and returns 400, then drains the echo before restarting. |
| Fixed alarm distances are inflexible | An **8-bit runtime threshold input** (`thr`) sets the alarm distance in cm. |
| Software polling wastes a processor | The module triggers, measures, and reports on its own, with a one-cycle `valid` pulse for each new result. |
| Different clock frequencies | All timing is **parameterized**, so the same RTL works at other clock rates by changing the parameters. |

---

## 3. Design Overview

### 3.1 Interface

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk` | input | 1 | System clock (design target 10 MHz) |
| `rst_n` | input | 1 | Synchronous, active-low reset |
| `en` | input | 1 | 1 = run measurements, 0 = stay idle |
| `echo` | input | 1 | Asynchronous ECHO pin of the sensor |
| `thr` | input | 8 | Alarm threshold in centimetres |
| `trig` | output | 1 | TRIG pin of the sensor (10 µs pulse) |
| `distance_cm` | output | 9 | Last measured distance in cm (0 to 400) |
| `valid` | output | 1 | One-cycle pulse when a new result is available |
| `err` | output | 1 | 1 = no echo or out of range |
| `alarm` | output | 1 | 1 = object closer than `thr` |

The module has **25 signal pins** in total (12 inputs and 13 outputs).

### 3.2 Parameters (defaults for a 10 MHz clock, 100 ns period)

| Parameter | Default | Meaning |
|---|---|---|
| `TRIG_CYCLES` | 100 | 100 × 100 ns = 10 µs trigger pulse |
| `CLKS_PER_CM` | 580 | 580 × 100 ns = 58 µs, the sound round-trip time per cm |
| `GAP_CYCLES` | 600000 | 60 ms between measurements (about 16 measurements per second) |
| `WAIT_CYCLES` | 300000 | 30 ms maximum wait for the echo to start |
| `MAX_CM` | 400 | Maximum range, the HC-SR04 limit |

### 3.3 Finite State Machine

| State | Code | Behaviour |
|---|---|---|
| `S_IDLE` | 0 | Outputs idle. Moves to `S_GAP` when `en = 1`. |
| `S_GAP` | 1 | Waits `GAP_CYCLES` between measurements, then raises `trig`. Returns to `S_IDLE` if `en` drops. |
| `S_TRIG` | 2 | Holds `trig` high for `TRIG_CYCLES`, then goes to `S_WAIT`. |
| `S_WAIT` | 3 | Waits for the synchronized echo to rise. On timeout (`WAIT_CYCLES`) it reports `err = 1`, `distance = 0`. |
| `S_MEAS` | 4 | While echo is high, counts clocks. Every `CLKS_PER_CM` clocks the `cm` counter increments. When echo falls, `distance_cm` is latched and `alarm = (cm < thr)`. If `cm` reaches `MAX_CM` it reports out-of-range and goes to `S_DRAIN`. |
| `S_DRAIN` | 5 | Waits for the echo to fall (or a timeout) before returning to `S_GAP`. |

### 3.4 Workflow of the Design (functional)

```
 en=1
  │
  ▼
[S_IDLE] ──► [S_GAP] ──(60 ms elapsed)──► [S_TRIG] ──(10 µs)──► [S_WAIT]
                ▲                                                  │
                │                              echo rises          │ no echo for 30 ms
                │                                   ▼              ▼
                │                             [S_MEAS]        err=1, dist=0
                │                         count clocks per cm        │
                │           echo falls ──►  latch distance, alarm     │
                │                      cm reaches 400 ─► [S_DRAIN]    │
                └─────────────────────────────────────────────────────┘
```

### 3.5 Engineering Workflow (RTL to routed layout)

```
Verilog RTL ──► RTL simulation (Icarus Verilog + GTKWave)
      │
      ▼
Logic synthesis (Yosys + ABC, sky130_fd_sc_hd) ──► gate-level netlist
      │
      ▼
Gate-level simulation (same testbench, sky130 cell models) ──► RTL vs GLS comparison
      │
      ▼
OpenLane v0.21 flow:  Synthesis ► Floorplan ► PDN ► Placement ► CTS ► Routing
      │
      ▼
Parasitic extraction (SPEF) ──► Multi-corner STA (ff / tt / ss) with OpenSTA
```

---

## 4. Design Features

- **Divider-free distance computation** using a cycle-per-centimetre tick counter.
- **Two-flip-flop synchronizer** on the asynchronous echo input.
- **Fully parameterized timing**, so the design is portable across clock frequencies.
- **Programmable alarm threshold** (`thr`, 0 to 255 cm). `thr = 0` never alarms, and the comparison is strict (`<`), so a reading equal to the threshold does not alarm.
- **Robust error handling**: no-echo timeout and out-of-range detection, each with a clean recovery to normal operation.
- **Single-cycle `valid` strobe**, which makes it easy to connect to a register, FIFO, or display driver.
- **Synchronous reset** and an **enable input** that stops triggering immediately.
- **Fixed minimum spacing** between measurements (60 ms) to avoid echo interference from the previous ping.
- Output-bit widths sized exactly: 20-bit timer, 12-bit sub-counter, 9-bit centimetre counter.

---

## 5. Verification Strategy

The self-checking testbench `tb_ultrasonic_ranger.v` uses **scaled-down parameters** (`CLKS_PER_CM=10`, `TRIG_CYCLES=10`, `GAP_CYCLES=50`, `WAIT_CYCLES=800`, `MAX_CM=400`) so that simulation finishes quickly. A **behavioural HC-SR04 model** answers every TRIG pulse with an echo whose width is `d × CLKS_PER_CM + 5` cycles, which should be read as exactly `d` cm.

| # | Scenario | Expected result |
|---|---|---|
| 1 | Reset behaviour | All outputs low during reset, no `trig` while `en = 0` |
| 2 | Normal measurements: 50, 150, 100, 99, 1, 0, 399 cm | Correct distance, `err = 0`, alarm only when below the threshold of 100 (100 cm does not alarm) |
| 3 | Threshold changes (200, 0, 255) | Alarm follows the new threshold; `thr = 0` never alarms |
| 4 | Out-of-range echo (4200 cycles) | `distance = 400`, `err = 1`, `alarm = 0` |
| 5 | Recovery after the error | Next reading (30 cm) is normal |
| 6 | No echo | Timeout: `distance = 0`, `err = 1`; recovery with 70 cm |
| 7 | `en = 0` | No trigger pulses while disabled; resumes when re-enabled (40 cm) |
| 8 | Reset in the middle of a measurement | Outputs clear, then the design works again (20 cm) |
| 9 | Measurement spacing | At least `GAP_CYCLES` between triggers |

The trigger width is also checked by an independent monitor. The testbench ends with a PASS/FAIL count and a watchdog timeout.

---




---

## 1. Phase 2: Environment Setup for OpenLane

OpenLane was run in a separate Ubuntu VirtualBox machine (`vsdsquadron`) provided with the workshop disk image `openlane.vdi` (30 GB). This section documents the setup and the first challenge encountered.




### 1.1 Checking Docker and OpenLane

```bash
docker --version                     # failed with "executable file not found" (see challenge 3)
unalias docker                       # removes the shell alias that was hijacking the command
type docker                          # -> docker is hashed (/usr/bin/docker)
type -a docker
sudo docker images                   # lists local images
find ~ -maxdepth 6 -name flow.tcl 2>/dev/null     # locate OpenLane
find ~ -maxdepth 6 -type d -name sky130A          # locate the PDK
which iverilog gtkwave yosys magic klayout netgen sta openroad
df -h /                              # check free disk space
free -h | head -2                    # check memory
```

| Command | Purpose |
|---|---|
| `unalias docker` | The VM had an alias that started a container with the typed words as the command, so `docker --version` and `docker build` produced "executable file not found". Removing it restores normal behaviour. |
| `sudo docker images` | Confirms the OpenLane image is present. |
| `find ... flow.tcl` | Locates the OpenLane folder (the one that contains `flow.tcl` and `designs`). |
| `find ... sky130A` | Locates the PDK folder. |
| `which ...` | Reports which tools are installed natively. |
| `df -h /`, `free -h` | Checks disk and memory; OpenLane needs several GB of free disk. |


### 1.2 Creating the design folder and configuration

The project files originally lived in the other VM, and drag-and-drop between VMs did not work. The RTL was recreated in the OpenLane VM by pasting it into a file.

```bash
cd ~/Desktop/work/tools/openlane_working_dir/openlane
mkdir -p designs/ultrasonic_ranger/src
# the RTL file ultrasonic_ranger.v was placed into designs/ultrasonic_ranger/src/
grep -n "parameter" designs/ultrasonic_ranger/src/ultrasonic_ranger.v   # confirm the real-hardware parameters
```

`config.json` (created with a here-document so no text editor was needed):

```bash
cat > designs/ultrasonic_ranger/config.json << 'EOF'
{
  "DESIGN_NAME": "ultrasonic_ranger",
  "VERILOG_FILES": "$::env(DESIGN_DIR)/src/ultrasonic_ranger.v",
  "CLOCK_PORT": "clk",
  "CLOCK_PERIOD": 100,
  "FP_CORE_UTIL": 40,
  "PL_TARGET_DENSITY": 0.5,
  "DIODE_INSERTION_STRATEGY": 3
}
EOF
```

| Key | Value | Meaning |
|---|---|---|
| `DESIGN_NAME` | `ultrasonic_ranger` | Top module name |
| `VERILOG_FILES` | `$::env(DESIGN_DIR)/src/...` | RTL location (the `dir::` form is not supported in v0.21, see challenge 6) |
| `CLOCK_PORT` | `clk` | Clock input |
| `CLOCK_PERIOD` | `100` | 100 ns = 10 MHz |
| `FP_CORE_UTIL` | `40` | Target core utilization of 40 % |
| `PL_TARGET_DENSITY` | `0.5` | Placement density target |
| `DIODE_INSERTION_STRATEGY` | `3` | Fake antenna-diode insertion |

The **default (real-hardware) parameters** are used in this flow, since the scaled-down values were only meant for fast simulation.

---

## 2. Phase 3: OpenLane Physical Design Flow

### 2.1 Starting the OpenLane container

```bash
export PDK_ROOT=~/Desktop/work/tools/openlane_working_dir/pdks
cd ~/Desktop/work/tools/openlane_working_dir/openlane
sudo docker run -it --rm \
    -v $(pwd):/openLANE_flow \
    -v $PDK_ROOT:$PDK_ROOT \
    -e PDK_ROOT=$PDK_ROOT \
    -u $(id -u):$(id -g) \
    efabless/openlane:v0.21
```

| Option | Purpose |
|---|---|
| `-it --rm` | Interactive terminal; the container is deleted on exit. |
| `-v $(pwd):/openLANE_flow` | Mounts the OpenLane folder, so all results are stored on the VM's disk and survive after the container exits. |
| `-v $PDK_ROOT:$PDK_ROOT -e PDK_ROOT=...` | Makes the SKY130 PDK visible to the flow. |
| `-u $(id -u):$(id -g)` | Runs as the normal user so generated files are not owned by root. |

Inside the container (prompt `bash-4.2$`), the flow is started interactively so every stage can be inspected:

```tcl
./flow.tcl -interactive
package require openlane 0.9
prep -design ultrasonic_ranger
```

`prep` creates the run folder `runs/22-10_19-32` and loads the PDK, the standard cell library and the configuration. After `Preparation complete`, the prompt changes to `%`.

### 2.2 Synthesis (`run_synthesis`)

```tcl
run_synthesis
```
This runs Yosys with OpenLane's own script (with ABC timing-driven mapping to the 100 ns clock) and then OpenSTA for a first timing check.



*What it shows and why:* The first run failed with `ERROR: Can't open input file 'dir::src/ultrasonic_ranger.v'`. OpenLane v0.21 does not understand the `dir::` prefix (used by newer OpenLane versions), so Yosys looked for a file literally named `dir::src/...`. Changing `VERILOG_FILES` to `$::env(DESIGN_DIR)/src/ultrasonic_ranger.v` solved it.












<img width="1680" height="893" alt="openlanesynthsuccess1" src="https://github.com/user-attachments/assets/51233a21-86fa-4dd2-9998-993243608f3e" />


<img width="1680" height="893" alt="openlanesynthsuccess2" src="https://github.com/user-attachments/assets/519ae6e6-5373-40cb-b528-b5827923693b" />

<img width="1680" height="893" alt="openlanesynthsuccess3" src="https://github.com/user-attachments/assets/b356e422-a535-4698-a33d-33ed2877289a" />

<img width="1680" height="893" alt="openlanesynthsuccess4" src="https://github.com/user-attachments/assets/66198f22-a9ce-4c4c-b400-c68002251781" />

<img width="1680" height="893" alt="openlanesynthsuccess5" src="https://github.com/user-attachments/assets/946b37a0-6919-489c-b53f-d96286bafc8f" />

<img width="1680" height="893" alt="openlanesynthsuccess6" src="https://github.com/user-attachments/assets/4d53c6c8-35bf-46ae-ae91-cda9fc3768ef" />

<img width="1680" height="893" alt="openlanesynthsuccess7" src="https://github.com/user-attachments/assets/39d8932f-5952-475a-87de-ffcc972c72e0" />

<img width="1680" height="893" alt="openlanesynthsuccess8" src="https://github.com/user-attachments/assets/72d58da6-bb50-4012-8baf-0c25d6d4a375" />


<img width="1680" height="893" alt="openlanesynthsuccess9" src="https://github.com/user-attachments/assets/aaeae660-b838-442e-abc7-5f7c9bd02e40" />































*What it shows and why:* After the fix, the log ends with `Synthesis was successful`, `tns 0.00`, `wns 0.00`. Key results:

| Item | Value |
|---|---|
| Tool | Yosys 0.9+3621 with ABC, then OpenSTA 2.3.0 |
| Library / corner | sky130_fd_sc_hd, tt_025C_1v80 |
| Constraints | 100 ns clock, 20 ns input and output delay (20 % of the period), 0.01765 pF output load |
| Total cells | **457** |
| Flip-flops | **62** × `sky130_fd_sc_hd__dfxtp_2` |
| Chip area | **4185.264 µm²** |
| ABC critical path | 3516.89 ps (about 3.5 ns), from `timer[0]`, through 14 logic levels |
| FSM encoding | Yosys re-encoded the 6-state FSM as **one-hot** (6 bits) |
| WNS / TNS | 0.00 / 0.00 |

*Why 62 flip-flops and not 59:* The standalone Yosys run kept the 3-bit binary state register. OpenLane's flow extracts the FSM and recodes it one-hot, which needs 6 flip-flops instead of 3: 59 − 3 + 6 = 62. The FSM transition table printed in the log lists the 22 transitions between the 6 states.

*Why the area (4185 µm²) is larger than the standalone run (about 2830 to 2840 µm²):* OpenLane maps to larger drive-strength cells (`_2` variants), inserts buffers (49 `buf_1`, 67 `inv_2`), and uses a timing-driven ABC script with the design constraints. The two numbers come from two different, valid flows.

*Warnings that are harmless:* `Wire ... is used but has no driver` for the output registers is a limitation of Yosys' check pass with library flip-flops. `ABC: Error: The network is combinational` is printed by ABC's retiming step when no sequential elements are present in the extracted network.

### 2.3 Floorplan, I/O placement, tap/decap insertion and PDN

```tcl
run_floorplan
```
In OpenLane v0.21 this one command also performs the I/O placement, tap/endcap insertion and PDN generation, so `gen_pdn` does not have to be run again.








<img width="1680" height="893" alt="floorplan" src="https://github.com/user-attachments/assets/3ad5af32-490a-4e0a-8cbd-1b790ec05000" />


*What it shows and why:* The log ends with `PDN generation was successful`. The yellow `PSM-xxxx` warnings only say that no voltage-pad location file or bump pitch was provided, so defaults were used. They do not affect the result.

| Item | Value |
|---|---|
| Die area | 0, 0 to 113.33 × 124.05 µm (about 14,058 µm²) |
| Core area | (5.52, 10.88) to (107.64, 111.52) µm, 102.12 × 100.64 µm (about 10,277 µm²) |
| Rows | 37 rows of 222 sites (row height 2.72 µm) |
| Initial utilization | **42.58 %** (4185 µm² of cells in a 10,277 µm² core) |
| I/O pins | 25 signal pins, placed randomly and evenly around the boundary |
| Endcaps / tapcells | 74 / 136 inserted |
| Components after taps | 667 (457 + 74 + 136) |
| Power nets | `VPWR`, `VGND` |
| PDN layers | met1 rails (width 0.48 µm, pitch 2.72 µm), met4 straps (1.6 µm width, 34.04 µm pitch), met5 straps (1.6 µm width, 33.547 µm pitch) |
| PDN nodes | 961 on VPWR, 755 on VGND, with all nodes connected |

*Why the warning "Current core area is too small for a power grid. Minimizing the power grid" appeared:* The core is only about 100 µm wide, which is smaller than the standard strap spacing, so OpenLane shrank the grid. The grid was still built and verified as connected.

*Why utilization is about 40 %:* It follows from `FP_CORE_UTIL = 40` in the configuration, and the actual value is slightly higher (42.58 %) because the core area is rounded to whole rows and sites.



### 2.4 Placement

```tcl
run_placement
```
This runs global placement (RePlAce), the resizer (buffer insertion and sizing), and detailed placement (legalization).


<img width="1680" height="893" alt="placement1" src="https://github.com/user-attachments/assets/ae286efc-40f2-440a-ab0a-4fb92430115a" />

<img width="1680" height="893" alt="placement2" src="https://github.com/user-attachments/assets/ee97726a-7416-4cda-8693-68b4f39cb546" />

<img width="1680" height="893" alt="placement3" src="https://github.com/user-attachments/assets/17d38b1a-7458-410f-8a46-059736fc46f8" />

<img width="1680" height="893" alt="placement4" src="https://github.com/user-attachments/assets/0646d4cd-a93d-4bdf-a7d8-3e4683c41a93" />

<img width="1680" height="893" alt="placement5" src="https://github.com/user-attachments/assets/6f8a44c2-796e-4b71-bc13-a564813259e1" />









| Item | Value |
|---|---|
| Placeable cells / fixed cells | 457 / 210 (74 endcaps + 136 taps) |
| Target density | 0.50 |
| Global placement | 370 Nesterov iterations, final overflow **0.098** (below the 0.1 convergence threshold) |
| Resizer | **11 input buffers and 13 output buffers** inserted, 361 instances resized |
| Instances after placement | 691 (667 + 24 buffers) |
| Final utilization | 41 % |
| Detailed placement | 0.0 µm displacement; HPWL **10,624 µm** (10,912.7 µm before the swap optimization, −2.6 %) |
| Pin count | 27 (the 25 signal pins plus `VPWR` and `VGND`) |
| Timing (ideal clock) | WNS 0.00, TNS 0.00; **setup slack +76.83 ns**, **hold slack +0.26 ns** |

*Why the results look like this:* The overflow dropping from 0.91 to 0.098 over the iterations is the normal convergence of the cells spreading from a central cluster to a legal density. Buffers are inserted on all 24 ports (11 inputs and 13 outputs) to isolate the core logic from the pins. The worst setup path starts at input `thr[6]` and ends at flip-flop `_799_`, with data arriving at 23.01 ns against a required time of 99.85 ns. About 20 ns of that is the input delay constraint. The worst hold path is `echo_sync[0]` to `_798_` (the reset gating `and2` of the synchronizer).

*Warning that is harmless:* `module sky130_fd_sc_hd__tapvpwrvgnd_1 not found. Creating black box` appears because tap cells have no logic, so STA treats them as empty boxes.


### 2.5 Clock tree synthesis

```tcl
run_cts
```
This runs TritonCTS to build a balanced H-tree on the `clk` net and then repairs and legalizes the placement.


<img width="1680" height="893" alt="cts1" src="https://github.com/user-attachments/assets/526f9c35-3728-4ae4-a94a-1bda2f8ec631" />
<img width="1680" height="893" alt="cts2" src="https://github.com/user-attachments/assets/1e783ec7-036a-408e-92ff-a11310083b9c" />
<img width="1680" height="893" alt="cts3" src="https://github.com/user-attachments/assets/5268a99a-a178-4610-8045-e129895e0992" />

<img width="1680" height="893" alt="cts4" src="https://github.com/user-attachments/assets/ca2ffcd7-f83c-4692-a9b6-36760b704840" />

<img width="1680" height="893" alt="cts5" src="https://github.com/user-attachments/assets/c71b34f8-85c3-4709-a983-b6093f3f6f3f" />
<img width="1680" height="893" alt="cts6" src="https://github.com/user-attachments/assets/0194ece7-15a8-4dd3-bdc2-af380be463ab" />






| Item | Value |
|---|---|
| Clock net / sinks | `clk` / **62 sinks** (all flip-flops) |
| Topology | H-tree, 3 levels (horizontal, vertical, horizontal), groups of 20 sinks, max diameter 50 µm |
| Clock buffers / nets created | **15 / 15** |
| Buffers per clock path | **4** (min and max both 4, so the tree is balanced) |
| Fanout distribution | 5:1, 6:1, 7:1, 8:3, 10:2 |
| Instances after CTS | 706 (691 + 15) |
| Utilization | 42 % |
| Legalization | 81.2 µm total displacement, 6.4 µm maximum |
| **Clock latency** | max 1.03 ns, min 0.39 ns |
| **Clock skew** | **0.64 ns** |
| Setup / hold slack | **+76.95 ns / +0.26 ns**, WNS 0.00, TNS 0.00 |

*Why the results look like this:* With 62 sinks and a cluster size of 20, TritonCTS needs a 3-level tree and 15 buffers. A balanced tree has the same buffer count in every path (4). The 0.64 ns skew is only 0.64 % of the 100 ns clock period, so it has no effect on timing. The standard cells changed from `_2` to `_1` drive strength after CTS (for example `dfxtp_2` to `dfxtp_1`) because the resizer downsized cells that had a large timing margin.

*Warnings that are harmless:* `6336 wires are pure wire`, `Creating fake entries in the LUT` and `default_operating_condition not found` appear for small designs.



### 2.6 Routing

```tcl
run_routing
```
This runs global routing (FastRoute), antenna repair, filler insertion, detailed routing (TritonRoute), SPEF parasitic extraction and a final STA.




<img width="1680" height="893" alt="routing" src="https://github.com/user-attachments/assets/ed31f598-23e3-434d-85bd-d6c37f2ad9d8" />


**Global routing**

| Item | Value |
|---|---|
| Nets routed | 508 |
| Overflow | **0** (no congestion) |
| Wirelength / vias | 21,058 µm / 2,507 |
| Layer usage | met2 19.71 %, met3 23.28 %, others 0 % |
| Antenna violations | **0** |

**Detailed routing**

| Item | Value |
|---|---|
| DRC violations per optimization pass | 241 → 69 → 61 → 2 → **0** |
| Final wirelength | **12,653 µm** (li1 4, met1 6,065, met2 6,349, met3 235) |
| Vias | **3,521** |
| Filler cells | 903 inserted (706 + 903 = 1,609 components) |
| Result | **"No DRC violations after detailed routing"** |
| Runtime | **34 min 9 s** |
| Post-route timing (STA in the flow) | WNS 0.00, TNS 0.00, setup slack +76.77 ns, hold slack +0.30 ns |

*Why the results look like this:* The detailed router starts with 241 violations (mostly short and spacing issues at the pin access points) and iteratively rips up and re-routes until none are left. The route uses almost only met1 and met2, because the design is small and uncongested (global routing overflow of 0). The wirelength after detailed routing (12,653 µm) is lower than the global-routing estimate (21,058 µm), since global routing counts grid-based segments. Fillers fill the unused row space so that the power rails and wells are continuous.


---

## 2.6 Multi-Corner Static Timing Analysis

The in-flow STA only covers a single corner, so STA was repeated with the extracted routing parasitics (`ultrasonic_ranger.spef`) for the three standard PVT corners, with the **clock treated as propagated**, so the real CTS latency and skew are included.

**Script `sta_corner.tcl`**
```tcl
read_liberty $::env(PDK)/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__$::env(CORNER).lib
read_verilog results/synthesis/ultrasonic_ranger.synthesis_preroute.v
link_design ultrasonic_ranger
read_spef results/routing/ultrasonic_ranger.spef
create_clock -name clk -period 100 [get_ports clk]
set_propagated_clock [get_clocks clk]
set_input_delay 20 -clock clk [get_ports {rst_n en echo thr*}]
set_output_delay 20 -clock clk [all_outputs]
set_load 0.01765 [all_outputs]
puts "===== CORNER: $::env(CORNER) ====="
report_checks -path_delay max -format full_clock_expanded
report_checks -path_delay min -format full_clock_expanded
puts "WNS:"; report_wns
puts "TNS:"; report_tns
puts "Worst setup slack:"; report_worst_slack -max
puts "Worst hold slack:"; report_worst_slack -min
report_clock_skew
```

**Commands**
```bash
cd ~/Desktop/work/tools/openlane_working_dir/openlane/designs/ultrasonic_ranger/runs/22-10_19-32
ls results/routing                    # confirm the SPEF file name
cat > sta_corner.tcl << 'EOF'
... (script above) ...
EOF

export PDK=~/Desktop/work/tools/openlane_working_dir/pdks
for c in tt_025C_1v80 ss_100C_1v60 ff_n40C_1v95; do
  CORNER=$c sta -no_splash -exit sta_corner.tcl > sta_$c.rpt 2>&1
done
grep -A1 -E "CORNER|Worst|WNS|TNS" sta_*.rpt
```

| Part | Purpose |
|---|---|
| `read_liberty ... $CORNER.lib` | Loads the timing library of the selected PVT corner. |
| `read_verilog ..._preroute.v` / `link_design` | Loads the final gate-level netlist (including CTS buffers and fillers). |
| `read_spef` | Back-annotates the real wire resistance and capacitance extracted from the routed layout. |
| `create_clock` / `set_propagated_clock` | Defines the 10 MHz clock and uses the real clock tree delay rather than an ideal clock. |
| `set_input_delay` / `set_output_delay` | 20 ns constraints matching the OpenLane flow. |
| `report_checks`, `report_wns/tns`, `report_worst_slack`, `report_clock_skew` | Produce the path reports, WNS, TNS, worst setup and hold slack, and skew. |
| `for c in ...` loop | Runs the same script for each of the three corners and saves one report each. |

<img width="1680" height="893" alt="sta1" src="https://github.com/user-attachments/assets/69c8b365-fa98-4b10-82fa-d9a758acbdd1" />


*What it shows:* The script was created by a here-document and `ls results/routing` confirmed the files `ultrasonic_ranger.def`, `.def.png`, `.def.ref`, `.spef` and `merged_unpadded.lef`. The SPEF file name was exactly as expected, so no edit was needed.


<img width="1680" height="893" alt="sta2" src="https://github.com/user-attachments/assets/f1842384-2499-4d2d-9941-268f9752a275" />


**Results**

| Corner | Process / Voltage / Temperature | Worst setup slack | Worst hold slack | WNS | TNS |
|---|---|---|---|---|---|
| `ff_n40C_1v95` | Fast / 1.95 V / −40 °C | **+79.27 ns** | **+0.27 ns** | 0.00 | 0.00 |
| `tt_025C_1v80` | Typical / 1.80 V / 25 °C | **+78.90 ns** | **+0.42 ns** | 0.00 | 0.00 |
| `ss_100C_1v60` | Slow / 1.60 V / 100 °C | **+77.79 ns** | **+0.90 ns** | 0.00 | 0.00 |

**Interpretation**

- **Timing is met at all PVT corners.** WNS and TNS are 0.00, which means there is no violating path anywhere in the design.
- **Setup slack is huge (about 78 ns of the 100 ns period).** The logic is shallow (the longest combinational path is a few ns), and at 10 MHz there is a lot of time. About 20 of the 22 ns of arrival time in the worst path is the specified 20 ns input delay, so the actual logic delay is only about 3 ns. The design could run much faster than 10 MHz.
- **Setup slack follows the expected trend**: it is smallest at the slow corner (77.79 ns at ss) and largest at the fast corner (79.27 ns at ff), because slow silicon (low voltage, high temperature) makes cells slower.
- **Hold slack is the tightest margin and is smallest at the fast corner** (+0.27 ns at ff), which is the expected pattern, since fast cells give short data paths. The worst hold paths are register-to-register paths through very little logic (for example the `echo_sync` synchronizer stage with its reset-gating AND). Because the clock is propagated here, this number includes the real 0.64 ns clock skew from CTS, and it is still positive at every corner.
- The three report files (`sta_ff_n40C_1v95.rpt`, `sta_tt_025C_1v80.rpt`, `sta_ss_100C_1v60.rpt`) contain the full path-by-path breakdown and the clock skew.

---

## 3. Challenges Faced and Solutions

| # | Challenge | Symptom | Cause | Solution |
|---|---|---|---|---|
| 1 | VM login screen never appeared | Console looped on `Started/Stopping User Manager for UID 121` | GDM crashing; video memory only 16 MB and the 30 GB disk nearly full | Increased video memory, tried safe-graphics boot options, then created a fresh VM from a freshly extracted `openlane.vdi` |
| 2 | Could not reach a text console | VirtualBox Input menu had no Ctrl-Alt-F3 entry and the Host-key combination did not work | Keyboard capture / VirtualBox menu limitations | Used recovery mode and GRUB edit options; replaced the VM |
| 3 | `docker` commands failed | `exec: "--version": executable file not found in $PATH` | A shell alias made `docker` run a container with the typed words as the command | `unalias docker`, verified with `type -a docker` and used `/usr/bin/docker` |
| 4 | Unnecessary OpenSTA build attempt | `docker build --file Dockerfile.ubuntu22.04` failed | The alias problem above; also, OpenSTA is already included in the OpenLane image | Skipped the build and used the tools already inside `efabless/openlane:v0.21` |
| 5 | `sudo` password rejected three times | Garbled prompt text and "3 incorrect password attempts" | Mistyped password (nothing is displayed while typing) | Retyped carefully, tested with `sudo -k` and `sudo echo ok` |
| 6 | Low disk space | `df -h /` showed 95 % used with only 1.5 GB free | 30 GB disk filled by workshop files, an old OpenLane copy and a zip archive | Removed the old copy and archive, cleared the apt cache and journal logs, reached 6.6 GB free |
| 7 | Project files were in a different VM | `cp: cannot stat ... No such file or directory` | RTL was created in the first VM; drag and drop and shared clipboard did not work between VMs | Pasted the Verilog into a new file inside the OpenLane VM and verified it with `grep -n "parameter"` |
| 8 | Text editor shortcuts did not work | `Ctrl+O` ignored in nano | Terminal/VM key capture | Saved with `Ctrl+X`, `Y`, `Enter`, and used `cat > file << 'EOF'` here-documents instead; removed the leftover `ultrasonic_ranger.v.save` file |
| 9 | Synthesis could not find the RTL | `Can't open input file 'dir::src/ultrasonic_ranger.v'` | OpenLane v0.21 does not support the `dir::` prefix used by newer versions | Changed `VERILOG_FILES` to `$::env(DESIGN_DIR)/src/ultrasonic_ranger.v` and deleted the failed run folder |
| 10 | Losing the `%` interactive prompt | Typing `exit` ended the session, and a hung terminal would not respond | `exit` closes the OpenLane session; a long-running command froze the shell | Restarted the container and reopened the existing run with `prep -design ultrasonic_ranger -tag 22-10_19-32` (without `-overwrite`, so the results were kept) |
| 11 | Area differs between flows | 2830 / 2840 µm² (standalone Yosys) vs 4185 µm² (OpenLane) | OpenLane uses a timing-driven ABC script, larger drive strengths, buffers and a one-hot FSM | Documented the reasons (Section 8.2) |

---

## 11. Results Summary

| Stage | Key result |
|---|---|
| RTL simulation | 50 / 50 checks passed |
| Standalone Yosys synthesis | 106 generic cells, 59 `dfxtp_1`, 2830 to 2840 µm² |
| Gate-level simulation | **50 PASS, 0 FAIL**; waveforms match RTL |
| OpenLane synthesis | 457 cells, 62 flip-flops (one-hot FSM), **4185.26 µm²**, WNS/TNS 0 |
| Floorplan | Die 113.33 × 124.05 µm, core 102.12 × 100.64 µm, 42.6 % utilization, 25 pins |
| PDN | met1/met4/met5 grid, connected on VPWR and VGND |
| Placement | 691 instances, overflow 0.098, HPWL 10,624 µm, 24 I/O buffers |
| CTS | 62 sinks, 15 buffers, balanced 4-buffer paths, **skew 0.64 ns** |
| Routing | **0 DRC violations**, 0 antenna violations, 12,653 µm wire, 3,521 vias |
| Multi-corner STA | **Setup ≥ +77.79 ns, hold ≥ +0.27 ns** at ff, tt and ss; WNS = TNS = 0 |

---

## 4. Scope, Limitations and Future Work

**Scope of this document:** RTL design, simulation, synthesis, gate-level simulation, floorplanning, placement, CTS, routing and multi-corner STA. The final sign-off stages (GDS generation, DRC, LVS and antenna checks with Magic and Netgen) are not covered here.

**Possible improvements**
- Add a moving-average filter or median filter on `distance_cm` to remove sensor noise.
- Add a hysteresis band to the alarm to prevent chattering near the threshold.
- Add a binary-to-BCD converter and a 7-segment/LCD driver.
- Parameterize the speed of sound with a temperature input for a more accurate cm conversion.
- Raise the clock frequency or reduce the area by using the available timing margin.
- Complete the sign-off checks (DRC, LVS, antenna) and run a post-layout simulation with SDF annotation.

---

## 5. Tools and Environment

| Tool | Version / Notes |
|---|---|
| Host / virtualization | Oracle VirtualBox, Ubuntu 18.04 guest (`vsdsquadron`) |
| RTL simulation | Icarus Verilog (`iverilog 10.1`) and `vvp` |
| Waveform viewer | GTKWave |
| Logic synthesis | Yosys 0.47 (standalone), Yosys 0.9+3621 inside OpenLane |
| Physical design | OpenLane v0.21 (Docker image `efabless/openlane:v0.21`) |
| Placement / CTS / routing | OpenROAD 0.9.0, TritonCTS 2.0, TritonRoute, FastRoute |
| Timing analysis | OpenSTA 2.3.0 |
| Parasitic extraction | SPEF extractor included in OpenLane |
| PDK | SkyWater SKY130A, standard cell library `sky130_fd_sc_hd` |
| Layout screenshots | KLayout (run inside the OpenLane flow) |

---

## 6. Repository Structure

```
ultrasonic_ranger_project/
├── README.md
├── rtl/
│   └── ultrasonic_ranger.v
├── tb/
│   ├── tb_ultrasonic_ranger.v
│   └── tb_gls.v
├── synthesis/
│   └── ultrasonic_ranger_net.v
├── openlane/
│   └── designs/ultrasonic_ranger/
│       ├── config.json
│       └── src/ultrasonic_ranger.v
├── sta/
│   ├── sta_corner.tcl
│   ├── sta_ff_n40C_1v95.rpt
│   ├── sta_tt_025C_1v80.rpt
│   └── sta_ss_100C_1v60.rpt
└── screenshots/
    └── 01_grub_menu.png ... 30_multicorner_sta_results.png
```
