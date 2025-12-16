# RYB-team-8-2025-Code
# Rock Your Baby — Team 8 (2025) — Final Code

This repository contains our **Rock-Your-Baby (RYB)** controller implementation. The system closes the loop around the TU/e “cradle chair” technical model: it **reads baby-state indicators** (heartbeat + crying loudness) and **drives the cradle** (amplitude + frequency) using PWM signals, with the goal of **reducing the simulated baby’s internal stress level** over time.

---

## System overview

### What the cradle model provides (plant + inverse model)
The RYB set-up internally simulates a baby stress level **S ∈ [0, 100]** (not directly observable). The controller cannot read `S` directly; instead it must infer improvement/worsening using:

- **Crying volume H [%]** (responds *immediately* to stress but saturates at high stress)
- **Heart rhythm R [BPM]** via wrist LED flashes (responds to stress with a **delay τ**)

### What our controller outputs
The cradle motion is controlled via **two 1 kHz PWM signals**:
- One PWM sets **Frequency (F)**
- One PWM sets **Amplitude (A)**

The cradle expects **12 V PWM** capable of supplying **≥ 0.8 A**, so the motor submodule includes power interfacing (transistor/MOSFET driver stage). 

### Discrete motion regions (required mapping)
Both Frequency and Amplitude are **quantized into 5 regions** based on PWM duty-cycle:

| Region | Duty cycle target | Frequency (Hz) | Amplitude (%) |
|-------:|-------------------|----------------|---------------|
| 1 | ~0%  | 0.20 | 10 |
| 2 | ~20% | 0.35 | 20 |
| 3 | ~40% | 0.50 | 30 |
| 4 | ~60% | 0.65 | 40 |
| 5 | ~80% | 0.80 | 50 |

Duty cycle **> 90%** triggers a cradle warning/emergency indicator and must be avoided. :contentReference[oaicite:4]{index=4}

---

## Architecture

This repo is organized into **four embedded submodules** plus a **simulation/test** area:
├── decision/ # MASTER / decision-making controller (runs on PYNQ)
├── heartbeat/ # HEARTBEAT sensor module (wrist LED -> BPM estimate) (runs on PYNQ)
├── crying/ # CRYING sensor module (mic loudness -> crying metric) (runs on PYNQ)
├── motor/ # MOTOR driver module (A/F commands -> 1kHz PWM outputs) (runs on PYNQ)
└── sim/ # Simulation / Simulation created with expected baby behavior to test control logic (runs on pc)


### Communications model: UART ring protocol
Nodes communicate using a lightweight **UART “ring” protocol**:

- Each node has a unique address:
  - `0` = controller / master
  - `1` = heartbeat
  - `2` = crying
  - `3` = motor 

- Messages contain both destination and source and are forwarded unchanged until they reach the target.
- A node **does not forward its own message** if it receives it back (prevents endless circulation). 

> Practical wiring note: the ring can be connected in any order as long as every device has two UART neighbors and all grounds share a common ground.

---

## Control strategy (high level)

1. **Read sensors**
   - Heartbeat module estimates BPM from wrist LED flashes.
   - Crying module estimates loudness/cry-level from microphone conditioning.

2. **Detect improvement vs. worsening**
   - Crying reacts quickly; heartbeat is delayed by τ seconds, so both signals are used to reduce false decisions. :contentReference[oaicite:8]{index=8}

3. **Select next (A, F)**
   - We operate in the discrete 5×5 grid of amplitude/frequency regions.
   - The “inverse model” behavior is column-based (K-matrix style); the correct path is found by testing moves and keeping changes that improve the observed outputs. :contentReference[oaicite:9]{index=9}

4. **Drive cradle**
   - Motor module outputs the required **1 kHz PWM** for A and F and converts logic-level control to **12 V / 0.8 A** compatible actuation via a driver stage. 

---

## Hardware & safety

This is a real lab set-up that can be damaged by misuse. At minimum:

- Do **not** force the cradle by hand; only use cradle motors.
- Do **not** unplug mains, display, or power off the PC while operating.
- Do **not** remove the baby doll from the chair.
- Avoid adhesives for the wrist sensor mounting (use Velcro/cable ties/rubber bands).

### Power-up sequence (lab set-up)
1. Connect cradle system to mains.
2. Power the cradle PC (red power button, then PC power).
3. Control software starts automatically after Windows boots.
4. Use the cradle on/off button to enable movement; preset selects heartbeat mode; back panel potentiometer sets volume.

---

## Build & run (typical workflow)

> Exact steps can vary by your PYNQ image + toolchain, but the structure below matches how this repo is intended to be used.

### Prerequisites
- PYNQ board with `libpynq` available (UART, ADC, buttons, display).
- UART wiring for the ring bus (common ground shared).
- Motor driver hardware capable of producing 12 V PWM at required current. 

### Running on hardware
1. Build/flash each submodule onto its target (PYNQ for decision; sensor/actuator submodules on their respective boards).
2. Verify UART ring addressing (0/1/2/3) and physical ring connections.
3. Start sensor modules first, then start the decision module.
4. Confirm that:
   - Heartbeat and crying values update,
   - Motor receives (A,F) commands and outputs stable 1 kHz PWM,
   - Duty cycles stay within the valid region bands (never > 90%). :contentReference[oaicite:14]{index=14}

---

## TODOS:

- fix the heartbeat module sign off issue. (fixed, lars)

- Fix motor ping not working(fixed, kuzey)

- change entire display ui of slaves (if we want to win the prize ig)

- construct the valid path matrix(fixed)

- not improved logic to diagonal jump(fixed)

- improve wait on crying level(fixed)

- fix the outside screen boundry issue with some clever logic (maybe, haha fixed)

- implement a viable test decision making algorithm(fixed)

- Emergency stop button on every submodule if something goes wrong(especially the motor one)

- fix the jittery display(fixed by embracing it)

- press button to imitate improved.(fixed)


