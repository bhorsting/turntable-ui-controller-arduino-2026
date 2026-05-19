/**
 * Turntable UI / arm-lift logic controller — Arduino UNO
 *
 * Record-end sequence (needle safety):
 *   1) Motor off, arm lift UP (~ARM_LIFT_TRAVEL_MS), lift de-energised.
 *   2) Motor ON + reject solenoid ON for exactly SOLENOID_ENGAGE_MS (1.5 s); then solenoid
 *      only off — motor stays on so the mechanism can drive the arm home.
 *   3) When the arm is back on the rest post, PIN_IN_ARM_IN reads “at rest” (here:
 *      !armInStable with default ARM_IN_NOT_REST_ACTIVE) — motor OFF fully.
 *   4) If PIN_IN_RECORD_END is still active, wait until it clears before normal idle.
 *
 * H-bridge (D7/D8 only — no separate enable pin):
 *   A on, B off = UP   |   A off, B on = DOWN   |   both off = de-energised (coast)
 *   With OUTPUT_SIGNALS_ACTIVE_LOW, idle is both pins HIGH (both relays off).
 *
 * Inputs (INPUT_PULLUP by default; tune *_ACTIVE for your wiring):
 *   PIN_BTN_ARM_LIFT — latching manual lift: ON = command UP, OFF = command DOWN.
 *   PIN_BTN_REJECT — momentary reject.
 *   PIN_IN_ARM_IN — true while the arm is NOT at rest (off the rest post); false when
 *      the arm has returned and the rest switch is released / not “off rest”.
 *   PIN_IN_ARM_LIFT_DOWN — true at full DOWN travel of the actuator.
 *   PIN_IN_RECORD_END — end of record / run-out (active = need emergency lift).
 *
 * Outputs: H-bridge A/B (D7/D8), PIN_OUT_MOTOR_ENABLE, PIN_OUT_REJECT_SOLENOID.
 *
 * Serial (115200): logs debounced IN changes and STATE changes only.
 * LEDs: arm-lift LED mirrors PIN_BTN_ARM_LIFT; reject LED blinks from reject start until
 * arm is parked (arm_in inactive). Record-end return also blinks reject LED.
 *
 * NOTE: Original text mentioned stopping the lift when it “reaches PIN_BTN_ARM_LIFT”.
 * The user button is not a travel sensor. While lowering, we de-energise immediately when
 * PIN_IN_ARM_LIFT_DOWN is seen. Timed moves still stop after ARM_LIFT_TRAVEL_MS (~2 s).
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin map (Arduino UNO)
//
// RESERVED — do NOT use for relays/LEDs (USB serial breaks if used as GPIO):
//   D0 = RX, D1 = TX  (hardware Serial / USB on CH340 boards)
// Unused but special: A4=SDA, A5=SCL (I2C); D10–D13 also SPI if you use SD shield.
//
// Our outputs D7–D9, D10–D13 do NOT share pins with UART. D6 unused. Serial glitches when
// relays switch are usually electrical (coil noise, 5 V sag, ground bounce),
// not a firmware pin clash — use separate relay supply and flyback diodes.
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_BTN_ARM_LIFT = 2;
constexpr uint8_t PIN_BTN_REJECT = 3;
constexpr uint8_t PIN_IN_ARM_IN = 4;
constexpr uint8_t PIN_IN_ARM_LIFT_DOWN = 5;

constexpr uint8_t PIN_IN_RECORD_END = A0;

constexpr uint8_t PIN_LED_POWER = 13;
constexpr uint8_t PIN_LED_ARM_LIFT = 11;
constexpr uint8_t PIN_LED_REJECT = 10;

constexpr uint8_t PIN_OUT_HBRIDGE_A = 7;
constexpr uint8_t PIN_OUT_HBRIDGE_B = 8;
constexpr uint8_t PIN_OUT_MOTOR_ENABLE = 9;
constexpr uint8_t PIN_OUT_REJECT_SOLENOID = 12;

constexpr uint8_t BUTTONS_ACTIVE = LOW;
constexpr uint8_t SENSOR_DOWN_ACTIVE = HIGH;
constexpr uint8_t ARM_IN_NOT_REST_ACTIVE = HIGH;
constexpr uint8_t RECORD_END_ACTIVE = LOW;

/** If false: arm parked when PIN_IN_ARM_IN reads “at rest” (!armInStable). If true: parked when the pin reads active (rest switch pressed). */
constexpr bool ARM_HOME_IS_ARM_IN_ACTIVE = false;

constexpr unsigned long DEBOUNCE_MS = 40;
constexpr unsigned long ARM_LIFT_TRAVEL_MS = 2000;
constexpr unsigned long REJECT_STABLE_AFTER_LIFT_MS = 400;
/** All uses of PIN_OUT_REJECT_SOLENOID (record-end return + reject). */
constexpr unsigned long SOLENOID_ENGAGE_MS = 1500;

/** Full blink period for status LEDs (250 ms on / 250 ms off = 500 ms). */
constexpr unsigned long LED_FLASH_HALF_MS = 250;

constexpr bool LOG_STATE_REASONS = true;
constexpr bool LOG_INPUT_EDGES = true;
constexpr bool LOG_OUTPUT_REASONS = false;
/** ANSI clear screen before each log line (needs a terminal that supports ESC). */
constexpr bool LOG_CLEAR_SCREEN = true;

/**
 * Typical 5 V opto relay / H-bridge boards: IN pin LOW = relay ON, HIGH = relay OFF.
 * Idle must be HIGH on D7–D8 (both off). Do not drive both LOW on active-low relay boards.
 * Set false only if your driver uses HIGH = on (logic-level signals, not relay modules).
 */
constexpr bool OUTPUT_SIGNALS_ACTIVE_LOW = true;

struct OutCmdState {
  uint8_t hbridgeA;
  uint8_t hbridgeB;
  uint8_t motor;
  uint8_t solenoid;
  uint8_t ledLift;
  uint8_t ledReject;
};

static OutCmdState outCmd = {};
static OutCmdState prevOutLogged = {};
static const __FlashStringHelper *pendingStateReason = nullptr;

static uint8_t outActiveLevel() {
  return OUTPUT_SIGNALS_ACTIVE_LOW ? LOW : HIGH;
}

static uint8_t outInactiveLevel() {
  return OUTPUT_SIGNALS_ACTIVE_LOW ? HIGH : LOW;
}

static uint8_t outLevel(bool active) {
  return active ? outActiveLevel() : outInactiveLevel();
}

static bool outIsActive(uint8_t pinLevel) {
  return pinLevel == outActiveLevel();
}

/** Drive relay/load pins to OFF before pinMode(OUTPUT) to reduce upload/reset clatter. */
static void initLoadOutputsSafe() {
  const uint8_t off = outInactiveLevel();
  digitalWrite(PIN_OUT_HBRIDGE_A, off);
  digitalWrite(PIN_OUT_HBRIDGE_B, off);
  digitalWrite(PIN_OUT_MOTOR_ENABLE, off);
  digitalWrite(PIN_OUT_REJECT_SOLENOID, off);
  pinMode(PIN_OUT_HBRIDGE_A, OUTPUT);
  pinMode(PIN_OUT_HBRIDGE_B, OUTPUT);
  pinMode(PIN_OUT_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_OUT_REJECT_SOLENOID, OUTPUT);
}

// ---------------------------------------------------------------------------
// Human-readable Serial logging for outputs and states
// ---------------------------------------------------------------------------
static void serialClearScreen() {
  if (!LOG_CLEAR_SCREEN) {
    return;
  }
  Serial.write(27);
  Serial.print(F("[2J"));
  Serial.write(27);
  Serial.print(F("[H"));
}

static void logOutLine(unsigned long nowMs, const __FlashStringHelper *what,
                       const __FlashStringHelper *why) {
  if (!LOG_OUTPUT_REASONS) {
    return;
  }
  serialClearScreen();
  Serial.print(nowMs);
  Serial.print(F(" ms OUT: "));
  Serial.print(what);
  Serial.print(F(" — "));
  Serial.println(why);
}

static void logLiftOutputs(unsigned long nowMs, const __FlashStringHelper *why) {
  const bool changed = (outCmd.hbridgeA != prevOutLogged.hbridgeA) ||
                         (outCmd.hbridgeB != prevOutLogged.hbridgeB);
  if (!changed) {
    return;
  }

  const bool aOn = outIsActive(outCmd.hbridgeA);
  const bool bOn = outIsActive(outCmd.hbridgeB);
  if (!aOn && !bOn) {
    logOutLine(nowMs, F("Lift H-bridge DE-ENERGISED (A and B off)"), why);
  } else if (aOn && !bOn) {
    logOutLine(nowMs, F("Lift H-bridge UP (A on, B off)"), why);
  } else if (!aOn && bOn) {
    logOutLine(nowMs, F("Lift H-bridge DOWN (A off, B on)"), why);
  } else {
    logOutLine(nowMs, F("Lift H-bridge both A and B on — check wiring"), why);
  }

  prevOutLogged.hbridgeA = outCmd.hbridgeA;
  prevOutLogged.hbridgeB = outCmd.hbridgeB;
}

static void logMotorOutput(unsigned long nowMs, const __FlashStringHelper *why) {
  if (outCmd.motor == prevOutLogged.motor) {
    return;
  }
  logOutLine(nowMs, outIsActive(outCmd.motor) ? F("Turntable MOTOR relay ON")
                                              : F("Turntable MOTOR relay OFF"),
             why);
  prevOutLogged.motor = outCmd.motor;
}

static void logSolenoidOutput(unsigned long nowMs, const __FlashStringHelper *why) {
  if (outCmd.solenoid == prevOutLogged.solenoid) {
    return;
  }
  logOutLine(nowMs, outIsActive(outCmd.solenoid) ? F("Reject SOLENOID ON")
                                                 : F("Reject SOLENOID OFF"),
             why);
  prevOutLogged.solenoid = outCmd.solenoid;
}

static void logLedOutput(unsigned long nowMs, const __FlashStringHelper *name, uint8_t pin,
                         uint8_t level, uint8_t &prevLevel, const __FlashStringHelper *why) {
  if (level == prevLevel) {
    return;
  }
  if (!LOG_OUTPUT_REASONS) {
    return;
  }
  serialClearScreen();
  Serial.print(nowMs);
  Serial.print(F(" ms OUT: "));
  Serial.print(name);
  Serial.print(F(" D"));
  Serial.print(pin);
  Serial.print(level == HIGH ? F(" ON — ") : F(" OFF — "));
  Serial.println(why);
  prevLevel = level;
}

// ---------------------------------------------------------------------------
// Lift helpers — semantic: A on + B off = UP, A off + B on = DOWN (see OUTPUT_SIGNALS_ACTIVE_LOW)
// ---------------------------------------------------------------------------
static void armLiftHardwareOff(const __FlashStringHelper *why) {
  outCmd.hbridgeA = outInactiveLevel();
  outCmd.hbridgeB = outInactiveLevel();
  digitalWrite(PIN_OUT_HBRIDGE_A, outCmd.hbridgeA);
  digitalWrite(PIN_OUT_HBRIDGE_B, outCmd.hbridgeB);
  logLiftOutputs(millis(), why);
}

static void armLiftUp(const __FlashStringHelper *why) {
  outCmd.hbridgeA = outActiveLevel();
  outCmd.hbridgeB = outInactiveLevel();
  digitalWrite(PIN_OUT_HBRIDGE_A, outCmd.hbridgeA);
  digitalWrite(PIN_OUT_HBRIDGE_B, outCmd.hbridgeB);
  logLiftOutputs(millis(), why);
}

static void armLiftDown(const __FlashStringHelper *why) {
  outCmd.hbridgeA = outInactiveLevel();
  outCmd.hbridgeB = outActiveLevel();
  digitalWrite(PIN_OUT_HBRIDGE_A, outCmd.hbridgeA);
  digitalWrite(PIN_OUT_HBRIDGE_B, outCmd.hbridgeB);
  logLiftOutputs(millis(), why);
}

static void setMotorEnable(bool on, const __FlashStringHelper *why) {
  outCmd.motor = outLevel(on);
  digitalWrite(PIN_OUT_MOTOR_ENABLE, outCmd.motor);
  logMotorOutput(millis(), why);
}

static void setRejectSolenoid(bool on, const __FlashStringHelper *why) {
  outCmd.solenoid = outLevel(on);
  digitalWrite(PIN_OUT_REJECT_SOLENOID, outCmd.solenoid);
  logSolenoidOutput(millis(), why);
}

static bool armIsParkedAtRest(bool armInStable) {
  return ARM_HOME_IS_ARM_IN_ACTIVE ? armInStable : !armInStable;
}

// ---------------------------------------------------------------------------
static bool readDebounced(uint8_t pin, uint8_t stableLevel, bool &stable,
                          bool &lastRaw, unsigned long &lastChangeMs) {
  const bool raw = (digitalRead(pin) == stableLevel);
  const unsigned long t = millis();
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChangeMs = t;
  } else if ((t - lastChangeMs) >= DEBOUNCE_MS) {
    stable = raw;
  }
  return stable;
}

enum class AppState : uint8_t {
  IdleArmAtRest,
  SecurityLiftUp,
  Playing,
  RecordEndLiftTiming,
  RecordEndMotorSolenoid,
  RecordEndWaitArmHome,
  RecordEndWaitRecordClear,
  RejectLiftTiming,
  RejectWaitStable,
  RejectSolenoidPulse,
  ManualLiftUpTiming,
  ManualLiftDown,
};

static const __FlashStringHelper *stateName(AppState s) {
  switch (s) {
  case AppState::IdleArmAtRest:
    return F("IdleArmAtRest");
  case AppState::SecurityLiftUp:
    return F("SecurityLiftUp");
  case AppState::Playing:
    return F("Playing");
  case AppState::RecordEndLiftTiming:
    return F("RecordEndLiftTiming");
  case AppState::RecordEndMotorSolenoid:
    return F("RecordEndMotorSolenoid");
  case AppState::RecordEndWaitArmHome:
    return F("RecordEndWaitArmHome");
  case AppState::RecordEndWaitRecordClear:
    return F("RecordEndWaitRecordClear");
  case AppState::RejectLiftTiming:
    return F("RejectLiftTiming");
  case AppState::RejectWaitStable:
    return F("RejectWaitStable");
  case AppState::RejectSolenoidPulse:
    return F("RejectSolenoidPulse");
  case AppState::ManualLiftUpTiming:
    return F("ManualLiftUpTiming");
  case AppState::ManualLiftDown:
    return F("ManualLiftDown");
  }
  return F("?");
}

static bool liftIsEnergised() {
  const uint8_t a = (uint8_t)digitalRead(PIN_OUT_HBRIDGE_A);
  const uint8_t b = (uint8_t)digitalRead(PIN_OUT_HBRIDGE_B);
  return outIsActive(a) || outIsActive(b);
}

static bool ledFlashOn(unsigned long t) {
  return ((t / LED_FLASH_HALF_MS) & 1U) != 0;
}

static void logInputChange(unsigned long nowMs, const __FlashStringHelper *name,
                           bool v) {
  if (!LOG_INPUT_EDGES) {
    return;
  }
  serialClearScreen();
  Serial.print(nowMs);
  Serial.print(F(" ms IN "));
  Serial.print(name);
  Serial.print(F(" "));
  Serial.println(v ? F("true") : F("false"));
}

static bool rejectInProgress = false;
static bool rejectLedOutputHigh = false;

static bool inRejectIndicatorFlow(AppState s) {
  return s == AppState::RejectLiftTiming || s == AppState::RejectWaitStable ||
         s == AppState::RejectSolenoidPulse;
}

static bool inRecordEndArmReturnIndicator(AppState s) {
  return s == AppState::RecordEndMotorSolenoid || s == AppState::RecordEndWaitArmHome;
}

static AppState state = AppState::IdleArmAtRest;
static AppState prevLoggedState = AppState::IdleArmAtRest;
static unsigned long phaseStartMs = 0;

static void logStateChange(unsigned long nowMs, AppState newState,
                           const __FlashStringHelper *why) {
  if (!LOG_STATE_REASONS) {
    return;
  }
  serialClearScreen();
  Serial.print(nowMs);
  Serial.print(F(" ms STATE -> "));
  Serial.print(stateName(newState));
  Serial.print(F(" — "));
  Serial.println(why);
}

static void transitionState(AppState next, const __FlashStringHelper *why) {
  if (state != next) {
    state = next;
    pendingStateReason = why;
  }
}
/** One up/down stroke per latch position (avoids 2 s repeat while limit unchanged). */
static bool manualLiftUpCycleDone = false;
static bool manualLiftDownCycleDone = false;

static bool liftUpTimerActive(unsigned long now) {
  return (now - phaseStartMs) < ARM_LIFT_TRAVEL_MS;
}

/** Timed UP: run up until timer expires, then de-energise. */
static void runLiftTimedUp(unsigned long now, const __FlashStringHelper *why) {
  (void)now;
  if (!liftUpTimerActive(now)) {
    armLiftHardwareOff(F("lift UP stroke finished (2 s elapsed)"));
    return;
  }
  armLiftUp(why);
}

/**
 * DOWN: de-energise immediately when DOWN limit is reached; otherwise run down until
 * timer expires (failsafe).
 */
static void runLiftTimedDown(unsigned long now, bool reachedDown,
                            const __FlashStringHelper *why) {
  (void)now;
  if (reachedDown) {
    armLiftHardwareOff(F("arm lift reached DOWN limit switch"));
    return;
  }
  if ((now - phaseStartMs) >= ARM_LIFT_TRAVEL_MS) {
    armLiftHardwareOff(F("lift DOWN stroke finished (2 s elapsed)"));
    return;
  }
  armLiftDown(why);
}

void setup() {
  initLoadOutputsSafe();

  Serial.begin(115200);

  pinMode(PIN_BTN_ARM_LIFT, INPUT_PULLUP);
  pinMode(PIN_BTN_REJECT, INPUT_PULLUP);
  pinMode(PIN_IN_ARM_IN, INPUT_PULLUP);
  pinMode(PIN_IN_ARM_LIFT_DOWN, INPUT_PULLUP);
  pinMode(PIN_IN_RECORD_END, INPUT_PULLUP);

  pinMode(PIN_LED_POWER, OUTPUT);
  pinMode(PIN_LED_ARM_LIFT, OUTPUT);
  pinMode(PIN_LED_REJECT, OUTPUT);

  armLiftHardwareOff(F("power-on — lift outputs safe / off"));
  setMotorEnable(false, F("power-on — motor off"));
  setRejectSolenoid(false, F("power-on — solenoid off"));
  digitalWrite(PIN_LED_POWER, HIGH);
  outCmd.ledLift = LOW;
  outCmd.ledReject = LOW;
  digitalWrite(PIN_LED_ARM_LIFT, LOW);
  digitalWrite(PIN_LED_REJECT, LOW);
  prevOutLogged = outCmd;

  state = AppState::IdleArmAtRest;
  prevLoggedState = state;
  phaseStartMs = 0;

  serialClearScreen();
  Serial.println(F("boot — log on IN change and STATE change only (115200)"));
}

void loop() {
  static bool armLiftLatchStable = false;
  static bool armLiftLatchLastRaw = false;
  static unsigned long armLiftLatchLastChg = 0;
  static bool rejectStable = false;
  static bool rejectLastRaw = false;
  static unsigned long rejectLastChg = 0;
  static bool rejectPrevStable = false;

  static bool downStable = false;
  static bool downLastRaw = false;
  static unsigned long downLastChg = 0;

  static bool armInStable = false;
  static bool armInLastRaw = false;
  static unsigned long armInLastChg = 0;
  static bool armInPrevStable = false;

  static bool recordEndStable = false;
  static bool recordEndLastRaw = false;
  static unsigned long recordEndLastChg = 0;
  static bool recordEndPrevStable = false;

  const unsigned long now = millis();

  const bool armLiftLatchOn =
      readDebounced(PIN_BTN_ARM_LIFT, BUTTONS_ACTIVE, armLiftLatchStable,
                    armLiftLatchLastRaw, armLiftLatchLastChg);
  readDebounced(PIN_BTN_REJECT, BUTTONS_ACTIVE, rejectStable, rejectLastRaw,
                rejectLastChg);
  const bool reachedDown =
      readDebounced(PIN_IN_ARM_LIFT_DOWN, SENSOR_DOWN_ACTIVE, downStable, downLastRaw,
                    downLastChg);
  readDebounced(PIN_IN_ARM_IN, ARM_IN_NOT_REST_ACTIVE, armInStable, armInLastRaw,
                armInLastChg);
  readDebounced(PIN_IN_RECORD_END, RECORD_END_ACTIVE, recordEndStable, recordEndLastRaw,
                recordEndLastChg);

  static bool inputLogPrimed = false;
  static bool prevArmLiftLatch = false;
  static bool prevReject = false;
  static bool prevReachedDown = false;
  static bool prevArmIn = false;
  static bool prevRecordEnd = false;

  if (LOG_INPUT_EDGES) {
    if (!inputLogPrimed) {
      if (now >= DEBOUNCE_MS * 3u) {
        prevArmLiftLatch = armLiftLatchOn;
        prevReject = rejectStable;
        prevReachedDown = reachedDown;
        prevArmIn = armInStable;
        prevRecordEnd = recordEndStable;
        inputLogPrimed = true;
      }
    } else {
      if (armLiftLatchOn != prevArmLiftLatch) {
        logInputChange(now, F("arm_lift_latch"), armLiftLatchOn);
        prevArmLiftLatch = armLiftLatchOn;
      }
      if (rejectStable != prevReject) {
        logInputChange(now, F("reject_btn"), rejectStable);
        prevReject = rejectStable;
      }
      if (reachedDown != prevReachedDown) {
        logInputChange(now, F("arm_lift_down"), reachedDown);
        prevReachedDown = reachedDown;
      }
      if (armInStable != prevArmIn) {
        logInputChange(now, F("arm_in"), armInStable);
        prevArmIn = armInStable;
      }
      if (recordEndStable != prevRecordEnd) {
        logInputChange(now, F("record_end"), recordEndStable);
        prevRecordEnd = recordEndStable;
      }
    }
  }

  const bool armInFall = !armInStable && armInPrevStable;
  armInPrevStable = armInStable;
  (void)armInFall;

  const bool rejectPressEdge = rejectStable && !rejectPrevStable;
  rejectPrevStable = rejectStable;

  const bool recordEndRise = recordEndStable && !recordEndPrevStable;
  recordEndPrevStable = recordEndStable;

  static bool armLiftLatchPrevEdge = false;
  const bool latchRise = armLiftLatchOn && !armLiftLatchPrevEdge;
  const bool latchFall = !armLiftLatchOn && armLiftLatchPrevEdge;
  armLiftLatchPrevEdge = armLiftLatchOn;
  if (latchRise) {
    manualLiftUpCycleDone = false;
    manualLiftDownCycleDone = false;
  }
  if (latchFall) {
    manualLiftUpCycleDone = false;
    manualLiftDownCycleDone = false;
  }

  const bool inRecordEndRecovery =
      (state == AppState::RecordEndLiftTiming ||
       state == AppState::RecordEndMotorSolenoid ||
       state == AppState::RecordEndWaitArmHome ||
       state == AppState::RecordEndWaitRecordClear);

  auto beginRecordEndLift = [&]() {
    rejectInProgress = false;
    setMotorEnable(false, F("record end — stop platter motor for safety"));
    setRejectSolenoid(false, F("record end — solenoid off during initial lift"));
    phaseStartMs = now;
    transitionState(AppState::RecordEndLiftTiming,
                    F("record end input active — start safety lift"));
    armLiftUp(F("record end — H-bridge UP to clear needle"));
  };

  auto beginRejectSequence = [&]() {
    rejectInProgress = true;
    setRejectSolenoid(false, F("reject — solenoid off until lift is up and stable"));
    phaseStartMs = now;
    transitionState(AppState::RejectLiftTiming,
                    F("reject button — arm is out, start reject sequence"));
    armLiftUp(F("reject — H-bridge UP before solenoid pulse"));
  };

  if (recordEndRise) {
    beginRecordEndLift();
  }

  const bool requestReject =
      rejectPressEdge || (rejectStable && !rejectInProgress && armInStable);
  if (requestReject && !recordEndStable && !inRecordEndRecovery &&
      !inRejectIndicatorFlow(state)) {
    beginRejectSequence();
  }

  switch (state) {

  case AppState::IdleArmAtRest:
    setMotorEnable(false, F("idle at rest — motor off"));
    if (recordEndStable) {
      beginRecordEndLift();
      break;
    }
    if (rejectPressEdge) {
      beginRejectSequence();
      break;
    }
    if (armInStable) {
      phaseStartMs = now;
      transitionState(AppState::SecurityLiftUp,
                      F("arm left rest while idle — safety lift before play"));
      armLiftUp(F("arm is out — H-bridge UP before starting platter"));
      break;
    }
    if (armLiftLatchOn) {
      if (reachedDown && !manualLiftUpCycleDone) {
        phaseStartMs = now;
        transitionState(AppState::ManualLiftUpTiming,
                        F("arm lift switch ON and at DOWN limit — drive up"));
        armLiftUp(F("manual latch ON — one UP stroke from down limit"));
      } else {
        armLiftHardwareOff(F("manual latch ON — already up, lift held off"));
      }
    } else if (!reachedDown && !manualLiftDownCycleDone) {
      phaseStartMs = now;
      transitionState(AppState::ManualLiftDown,
                      F("arm lift switch OFF and not at down — drive down"));
      armLiftDown(F("manual latch OFF — one DOWN stroke toward limit"));
    } else {
      armLiftHardwareOff(F("idle — lift off, latch matches position"));
    }
    break;

  case AppState::ManualLiftDown:
    setMotorEnable(false, F("manual lift move — motor off"));
    if (recordEndStable) {
      armLiftHardwareOff(F("record end during manual down"));
      transitionState(AppState::RecordEndLiftTiming, F("record end overrides manual down"));
      phaseStartMs = now;
      armLiftUp(F("record end — lift UP"));
      break;
    }
    if (armInStable) {
      armLiftHardwareOff(F("arm went out during manual down"));
      phaseStartMs = now;
      transitionState(AppState::SecurityLiftUp, F("arm out — switch to security lift"));
      armLiftUp(F("security lift UP"));
      break;
    }
    if (rejectPressEdge) {
      armLiftHardwareOff(F("reject during manual down"));
      beginRejectSequence();
      break;
    }
    if (armLiftLatchOn) {
      manualLiftDownCycleDone = false;
      phaseStartMs = now;
      transitionState(AppState::ManualLiftUpTiming, F("latch turned ON during down stroke"));
      armLiftUp(F("manual latch ON — switch to UP"));
      break;
    }
    runLiftTimedDown(now, reachedDown, F("manual down stroke in progress"));
    if (!liftIsEnergised()) {
      manualLiftDownCycleDone = true;
      transitionState(AppState::IdleArmAtRest, F("manual down stroke complete"));
    }
    break;

  case AppState::ManualLiftUpTiming:
    setMotorEnable(false, F("manual lift move — motor off"));
    if (recordEndStable) {
      armLiftHardwareOff(F("record end during manual up"));
      transitionState(AppState::RecordEndLiftTiming, F("record end overrides manual up"));
      phaseStartMs = now;
      armLiftUp(F("record end — lift UP"));
      break;
    }
    if (armInStable) {
      armLiftHardwareOff(F("arm went out during manual up"));
      phaseStartMs = now;
      transitionState(AppState::SecurityLiftUp, F("arm out — switch to security lift"));
      armLiftUp(F("security lift UP"));
      break;
    }
    if (rejectPressEdge) {
      armLiftHardwareOff(F("reject during manual up"));
      beginRejectSequence();
      break;
    }
    runLiftTimedUp(now, F("manual UP stroke in progress"));
    if (!liftUpTimerActive(now) && !liftIsEnergised()) {
      if (armLiftLatchOn) {
        manualLiftUpCycleDone = true;
        transitionState(AppState::IdleArmAtRest, F("manual UP done, latch still ON"));
      } else {
        manualLiftDownCycleDone = false;
        phaseStartMs = now;
        transitionState(AppState::ManualLiftDown, F("manual UP done, latch OFF — go down"));
        armLiftDown(F("manual latch OFF — start DOWN stroke"));
      }
    }
    break;

  case AppState::SecurityLiftUp:
    setMotorEnable(false, F("security lift — motor off until playing"));
    if (recordEndStable) {
      phaseStartMs = now;
      transitionState(AppState::RecordEndLiftTiming, F("record end during security lift"));
      armLiftUp(F("record end — lift UP"));
      break;
    }
    if (!armInStable) {
      armLiftHardwareOff(F("arm returned to rest during security lift"));
      transitionState(AppState::IdleArmAtRest, F("arm back on rest post"));
      break;
    }
    if (rejectPressEdge) {
      armLiftHardwareOff(F("reject during security lift"));
      beginRejectSequence();
      break;
    }
    runLiftTimedUp(now, F("security lift UP stroke in progress"));
    if (!liftIsEnergised()) {
      if (armInStable && !recordEndStable) {
        transitionState(AppState::Playing, F("lift up done — arm still out, start play"));
        setMotorEnable(true, F("playing — enable platter motor"));
      } else {
        transitionState(AppState::IdleArmAtRest, F("lift up done — return to idle"));
      }
    }
    break;

  case AppState::Playing:
    if (recordEndStable) {
      setMotorEnable(false, F("record end while playing"));
      phaseStartMs = now;
      transitionState(AppState::RecordEndLiftTiming, F("record end — stop play, lift arm"));
      armLiftUp(F("record end — H-bridge UP"));
      break;
    }
    if (!armInStable) {
      setMotorEnable(false, F("arm returned to rest — stop motor"));
      armLiftHardwareOff(F("arm on rest — lift off"));
      transitionState(AppState::IdleArmAtRest, F("arm parked during play"));
      break;
    }
    if (rejectPressEdge) {
      setMotorEnable(false, F("reject while playing — stop motor"));
      beginRejectSequence();
      break;
    }
    setMotorEnable(true, F("playing — platter motor on"));
    break;

  case AppState::RecordEndLiftTiming:
    runLiftTimedUp(now, F("record end — lift UP stroke"));
    if (!liftIsEnergised()) {
      if (recordEndStable) {
        armLiftHardwareOff(F("record end lift done — hand off to return"));
        phaseStartMs = now;
        transitionState(AppState::RecordEndMotorSolenoid,
                        F("lift up done — start motor + solenoid return"));
        setMotorEnable(true, F("record end return — motor on to drive arm home"));
        setRejectSolenoid(true, F("record end return — solenoid on 1.5 s"));
      } else if (armInStable) {
        phaseStartMs = now;
        transitionState(AppState::SecurityLiftUp, F("record end cleared, arm still out"));
        armLiftUp(F("security lift UP"));
      } else {
        transitionState(AppState::IdleArmAtRest, F("record end lift done — idle"));
      }
    }
    break;

  case AppState::RecordEndMotorSolenoid:
    armLiftHardwareOff(F("record end return — lift off during motor return"));
    setMotorEnable(true, F("record end return — motor driving arm home"));
    if ((now - phaseStartMs) < SOLENOID_ENGAGE_MS) {
      setRejectSolenoid(true, F("record end return — solenoid engaged"));
    } else {
      setRejectSolenoid(false, F("record end return — solenoid off after 1.5 s"));
      transitionState(AppState::RecordEndWaitArmHome,
                      F("solenoid pulse done — wait for arm on rest"));
    }
    break;

  case AppState::RecordEndWaitArmHome:
    armLiftHardwareOff(F("waiting for arm home — lift off"));
    setRejectSolenoid(false, F("waiting for arm home — solenoid off"));
    setMotorEnable(true, F("waiting for arm home — motor still on"));
    if (armIsParkedAtRest(armInStable)) {
      setMotorEnable(false, F("arm home — motor off"));
      if (recordEndStable) {
        transitionState(AppState::RecordEndWaitRecordClear,
                        F("arm home but record end still active"));
      } else {
        transitionState(AppState::IdleArmAtRest, F("arm home — reject/record cycle done"));
      }
    }
    break;

  case AppState::RecordEndWaitRecordClear:
    armLiftHardwareOff(F("waiting for record end to clear — lift off"));
    setMotorEnable(false, F("waiting for record end — motor off"));
    setRejectSolenoid(false, F("waiting for record end — solenoid off"));
    if (!recordEndStable) {
      transitionState(AppState::IdleArmAtRest, F("record end input cleared"));
    }
    break;

  case AppState::RejectLiftTiming:
    if (recordEndStable) {
      phaseStartMs = now;
      transitionState(AppState::RecordEndLiftTiming, F("record end during reject lift"));
      armLiftUp(F("record end — lift UP"));
      break;
    }
    runLiftTimedUp(now, F("reject — lift UP stroke"));
    if (!liftIsEnergised()) {
      phaseStartMs = now;
      transitionState(AppState::RejectWaitStable, F("reject lift done — wait before solenoid"));
    }
    break;

  case AppState::RejectWaitStable:
    armLiftHardwareOff(F("reject — lift up done, holding before solenoid"));
    if (recordEndStable) {
      beginRecordEndLift();
      break;
    }
    if ((now - phaseStartMs) >= REJECT_STABLE_AFTER_LIFT_MS) {
      phaseStartMs = now;
      transitionState(AppState::RejectSolenoidPulse, F("reject — stable, fire solenoid"));
      setRejectSolenoid(true, F("reject — solenoid ON for 1.5 s"));
    }
    break;

  case AppState::RejectSolenoidPulse:
    if (recordEndStable) {
      setRejectSolenoid(false, F("record end during reject solenoid"));
      beginRecordEndLift();
      break;
    }
    if ((now - phaseStartMs) >= SOLENOID_ENGAGE_MS) {
      setRejectSolenoid(false, F("reject — solenoid OFF after 1.5 s"));
      if (armInStable) {
        phaseStartMs = now;
        transitionState(AppState::SecurityLiftUp,
                        F("reject done — arm still out, security lift"));
        armLiftUp(F("reject — H-bridge UP after solenoid"));
      } else {
        transitionState(AppState::IdleArmAtRest, F("reject complete — arm on rest"));
      }
    }
    break;
  }

  outCmd.ledLift = armLiftLatchOn ? HIGH : LOW;
  digitalWrite(PIN_LED_ARM_LIFT, outCmd.ledLift);
  logLedOutput(now, F("Arm lift LED"), PIN_LED_ARM_LIFT, outCmd.ledLift, prevOutLogged.ledLift,
               armLiftLatchOn ? F("mirrors arm lift switch ON")
                              : F("mirrors arm lift switch OFF"));

  if (rejectInProgress && armIsParkedAtRest(armInStable)) {
    rejectInProgress = false;
  }

  if (rejectInProgress) {
    rejectLedOutputHigh = true;
  } else if (inRecordEndArmReturnIndicator(state)) {
    rejectLedOutputHigh = ledFlashOn(now);
  } else {
    rejectLedOutputHigh = false;
  }
  outCmd.ledReject = rejectLedOutputHigh ? HIGH : LOW;
  digitalWrite(PIN_LED_REJECT, outCmd.ledReject);
  if (rejectInProgress) {
    logLedOutput(now, F("Reject LED"), PIN_LED_REJECT, outCmd.ledReject, prevOutLogged.ledReject,
                 F("reject in progress — LED steady on"));
  } else if (inRecordEndArmReturnIndicator(state)) {
    logLedOutput(now, F("Reject LED"), PIN_LED_REJECT, outCmd.ledReject, prevOutLogged.ledReject,
                 rejectLedOutputHigh ? F("record end return — LED blink on phase")
                                   : F("record end return — LED blink off phase"));
  } else {
    logLedOutput(now, F("Reject LED"), PIN_LED_REJECT, outCmd.ledReject, prevOutLogged.ledReject,
                 F("reject LED off"));
  }

  if (LOG_STATE_REASONS && state != prevLoggedState) {
    logStateChange(now, state,
                   pendingStateReason ? pendingStateReason : F("(reason not recorded)"));
    pendingStateReason = nullptr;
    prevLoggedState = state;
  }
}
