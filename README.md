# Turntable UI Controller (Arduino UNO)

Firmware for turntable arm lift, platter motor, reject solenoid, and panel LEDs. Logic lives in `src/main.cpp`; pin wiring is in `docs/Pin-Wiring-Guide.html`.

Serial monitor: **115200 baud** (state changes, output reasons, I/O snapshot every 3 s).

---

## Outputs (pins)

| Pin | Signal | Role |
|-----|--------|------|
| D7 | H-bridge A | Direction A |
| D8 | H-bridge B | Direction B (opposite of A while moving) |
| D6 | — | **Unused** (no separate lift enable) |
| D9 | Motor enable | Platter motor relay |
| D12 | Reject solenoid | Reject / return solenoid |
| D10 | Reject LED | Panel indicator |
| D11 | Arm lift LED | Mirrors arm-lift latch switch |
| D13 | Power LED | On at boot (also UNO onboard LED) |

### Relay polarity (`OUTPUT_SIGNALS_ACTIVE_LOW = true`, default)

Typical opto relay boards: **pin LOW = relay ON**, **HIGH = OFF**. Tables below use **semantic** names (on/off). Pin levels for the default setting:

| Semantic | D7 A | D8 B | D9 motor | D12 solenoid |
|----------|------|------|----------|--------------|
| **Off (de-energised)** | HIGH | HIGH | HIGH | HIGH |
| **Lift UP** | LOW | HIGH | — | — |
| **Lift DOWN** | HIGH | LOW | — | — |
| **Motor on** | — | — | — | LOW | — |
| **Solenoid on** | — | — | — | — | LOW |

Panel LEDs (D10, D11, D13) are **active HIGH** (HIGH = LED on), not inverted.

### Lift combinations (semantic)

De-energised = **both A and B off** (no direction relay on). Do not drive both A and B **on** at once.

| Mode | A | B |
|------|---|---|
| Off | off | off |
| UP | on | off |
| DOWN | off | on |

With `OUTPUT_SIGNALS_ACTIVE_LOW = false`, “off” is **D7 LOW + D8 LOW** on the pins.

Timed UP runs for `ARM_LIFT_TRAVEL_MS` (2 s), then de-energises. Timed DOWN stops immediately when the down-limit input is active, or after 2 s as a failsafe.

---

## Status LEDs (every state, end of `loop`)

These apply in **all** states after the state `switch` runs:

| Output | Behavior |
|--------|----------|
| **Arm lift LED (D11)** | HIGH while arm-lift **latch** is ON; LOW when latch OFF |
| **Reject LED (D10)** | **Steady ON** while `rejectInProgress` (from reject start until arm is parked at rest). **Blinks** (500 ms period) during `RecordEndMotorSolenoid` and `RecordEndWaitArmHome`. Otherwise OFF |
| **Power LED (D13)** | HIGH after boot (unchanged in loop) |

---

## States and steady outputs

Outputs listed are what the state **drives each loop** while it remains in that state. Transitions may pulse outputs once when entering the next state. Record-end and reject can **pre-empt** other states (see notes).

### `IdleArmAtRest`

Arm on rest post; normal idle.

| D7–D8 lift | D9 motor | D12 solenoid | Notes |
|------------|----------|--------------|-------|
| **Off** | **Off** | **Off** | Default |

Also in this state (no state change):

- **Latch ON** + at down limit → one timed **UP** stroke → `ManualLiftUpTiming`
- **Latch ON** + not at down limit → lift stays **off**
- **Latch OFF** + not at down limit → one **DOWN** stroke → `ManualLiftDown`
- **Arm leaves rest** (`arm_in` active) → `SecurityLiftUp` with **UP**
- **Record end** rises → `RecordEndLiftTiming`
- **Reject** edge (or held reject with arm out) → `RejectLiftTiming`

---

### `ManualLiftDown`

Manual latch OFF: lowering toward down-limit switch.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **DOWN** (until limit or 2 s), then **off** | **Off** | **Off** |

Exits to `IdleArmAtRest` when lift de-energised. Record end / arm out / reject can redirect.

---

### `ManualLiftUpTiming`

Manual latch ON: timed UP stroke (~2 s).

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **UP** while timer active, then **off** | **Off** | **Off** |

When UP finishes: latch still ON → `IdleArmAtRest`; latch OFF → `ManualLiftDown` with **DOWN**.

---

### `SecurityLiftUp`

Arm moved off rest while idle; safety lift before play.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **UP** while timer active, then **off** | **Off** during lift; **on** only when entering `Playing` | **Off** |

When UP finishes: arm still out → `Playing` (**motor on**); else → `IdleArmAtRest`. Arm returns to rest during lift → lift **off**, `IdleArmAtRest`.

---

### `Playing`

Platter running; arm expected off rest post.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **Off** | **On** | **Off** |

Record end → lift **UP** (`RecordEndLiftTiming`), motor **off**. Reject → reject sequence, motor **off**. Arm returns to rest → motor **off**, `IdleArmAtRest`.

---

### `RecordEndLiftTiming`

End-of-record safety: lift arm UP before return sequence.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **UP** while timer active, then **off** | **Off** during lift; **on** when advancing to return | **Off** during lift; **on** when advancing to return |

When UP finishes and record end still active → `RecordEndMotorSolenoid` (motor + solenoid **on**). Other branches: `SecurityLiftUp` or `IdleArmAtRest`.

---

### `RecordEndMotorSolenoid`

Return stroke: motor drives arm home; solenoid pulsed.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **Off** | **On** | **On** for `SOLENOID_ENGAGE_MS` (1.5 s), then **off** |

After 1.5 s → `RecordEndWaitArmHome`. Reject LED **blinks** in this state.

---

### `RecordEndWaitArmHome`

Motor on until arm is back on rest post.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **Off** | **On** until arm parked, then **off** | **Off** |

Arm home + record end still active → `RecordEndWaitRecordClear`. Arm home + record end clear → `IdleArmAtRest`. Reject LED **blinks**.

---

### `RecordEndWaitRecordClear`

Arm home but record-end input still active.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **Off** | **Off** | **Off** |

When record end clears → `IdleArmAtRest`.

---

### `RejectLiftTiming`

Reject: lift UP before solenoid (arm was out).

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **UP** while timer active, then **off** | Unchanged from prior state until sequence continues | **Off** |

`rejectInProgress` set; reject LED **steady on**. When UP done → `RejectWaitStable`. Record end can override to record-end sequence.

---

### `RejectWaitStable`

Short hold after lift UP before solenoid fire.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **Off** | Typically **off** (not set here) | **Off** |

After `REJECT_STABLE_AFTER_LIFT_MS` (400 ms) → `RejectSolenoidPulse` with solenoid **on**.

---

### `RejectSolenoidPulse`

Reject solenoid engaged.

| D7–D8 lift | D9 motor | D12 solenoid |
|------------|----------|--------------|
| **Off** | Not driven in this state | **On** for 1.5 s, then **off** |

After 1.5 s: arm still out → `SecurityLiftUp` with **UP**; arm on rest → `IdleArmAtRest`. `rejectInProgress` clears when arm is parked (reject LED off).

---

## Priority / pre-emption

| Condition | Effect |
|-----------|--------|
| **Record end** (rising edge) | Starts or restarts record-end lift; blocks new reject requests while in record-end recovery states |
| **Record end** during reject | Aborts reject solenoid path; switches to record-end sequence |
| **Reject** | Blocked during record-end recovery; can start from global check when arm is out and not in reject-indicator states |

---

## State flow (overview)

```text
IdleArmAtRest
  ├─ arm out ──────────────► SecurityLiftUp ──► Playing ──► (record end / reject / arm home)
  ├─ latch manual ─────────► ManualLiftUpTiming / ManualLiftDown
  ├─ record end ───────────► RecordEndLiftTiming ──► RecordEndMotorSolenoid
  │                                              ──► RecordEndWaitArmHome ──► Idle (or WaitRecordClear)
  └─ reject ───────────────► RejectLiftTiming ──► RejectWaitStable ──► RejectSolenoidPulse
                                                                  ──► SecurityLiftUp or Idle
```

---

## Build / upload

```bash
python -m platformio run -t upload
python -m platformio device monitor -b 115200
```

See `platformio.ini` for board (`uno`) and port settings.
