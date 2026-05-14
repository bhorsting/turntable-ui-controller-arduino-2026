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
 * H-bridge direction (while lift is energised):
 *   A HIGH, B LOW  = arm lift UP   (raise / extend)
 *   A LOW, B HIGH  = arm lift DOWN (lower / retract)
 *
 * Inputs (INPUT_PULLUP by default; tune *_ACTIVE for your wiring):
 *   PIN_BTN_ARM_LIFT — latching manual lift (used mainly while arm is at rest).
 *   PIN_BTN_REJECT — momentary reject.
 *   PIN_IN_ARM_IN — true while the arm is NOT at rest (off the rest post); false when
 *      the arm has returned and the rest switch is released / not “off rest”.
 *   PIN_IN_ARM_LIFT_DOWN — true at full DOWN travel of the actuator.
 *   PIN_IN_RECORD_END — end of record / run-out (active = need emergency lift).
 *
 * Outputs: lift energise + H-bridge A/B, PIN_OUT_MOTOR_ENABLE, PIN_OUT_REJECT_SOLENOID.
 *
 * Serial (115200): every AppState transition is logged; debounced input changes are
 * logged as IN name true or false.
 * LEDs: arm-lift LED flashes (500 ms period) while the lift is energised; reject LED
 * flashes during user reject (lift → solenoid) and during record-end arm-return (motor
 * + wait-home). Reject is ignored during record-end recovery and during an active reject
 * sequence.
 *
 * NOTE: Original text mentioned stopping the lift when it “reaches PIN_BTN_ARM_LIFT”.
 * The user button is not a travel sensor. While lowering, we de-energise immediately when
 * PIN_IN_ARM_LIFT_DOWN is seen. Timed moves still stop after ARM_LIFT_TRAVEL_MS (~2 s).
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin map
// ---------------------------------------------------------------------------
constexpr uint8_t PIN_BTN_ARM_LIFT = 2;
constexpr uint8_t PIN_BTN_REJECT = 3;
constexpr uint8_t PIN_IN_ARM_IN = 4;
constexpr uint8_t PIN_IN_ARM_LIFT_DOWN = 5;

constexpr uint8_t PIN_IN_RECORD_END = A0;

constexpr uint8_t PIN_LED_POWER = 13;
constexpr uint8_t PIN_LED_ARM_LIFT = 11;
constexpr uint8_t PIN_LED_REJECT = 10;

constexpr uint8_t PIN_OUT_ENERGISE_LIFT = 6;
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

// ---------------------------------------------------------------------------
// Lift helpers — A HIGH B LOW = UP, A LOW B HIGH = DOWN
// ---------------------------------------------------------------------------
static void armLiftHardwareOff() {
  digitalWrite(PIN_OUT_ENERGISE_LIFT, LOW);
  digitalWrite(PIN_OUT_HBRIDGE_A, LOW);
  digitalWrite(PIN_OUT_HBRIDGE_B, LOW);
}

static void armLiftUp() {
  digitalWrite(PIN_OUT_ENERGISE_LIFT, HIGH);
  digitalWrite(PIN_OUT_HBRIDGE_A, HIGH);
  digitalWrite(PIN_OUT_HBRIDGE_B, LOW);
}

static void armLiftDown() {
  digitalWrite(PIN_OUT_ENERGISE_LIFT, HIGH);
  digitalWrite(PIN_OUT_HBRIDGE_A, LOW);
  digitalWrite(PIN_OUT_HBRIDGE_B, HIGH);
}

static void setMotorEnable(bool on) {
  digitalWrite(PIN_OUT_MOTOR_ENABLE, on ? HIGH : LOW);
}

static void setRejectSolenoid(bool on) {
  digitalWrite(PIN_OUT_REJECT_SOLENOID, on ? HIGH : LOW);
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
  return digitalRead(PIN_OUT_ENERGISE_LIFT) == HIGH;
}

static bool ledFlashOn(unsigned long t) {
  return ((t / LED_FLASH_HALF_MS) & 1U) != 0;
}

static void logInputChange(unsigned long nowMs, const __FlashStringHelper *name,
                           bool v) {
  Serial.print(nowMs);
  Serial.print(F(" ms IN "));
  Serial.print(name);
  Serial.println(v ? F("true") : F("false"));
}

static bool inRejectIndicatorFlow(AppState s) {
  return s == AppState::RejectLiftTiming || s == AppState::RejectWaitStable ||
         s == AppState::RejectSolenoidPulse;
}

static bool inRecordEndArmReturnIndicator(AppState s) {
  return s == AppState::RecordEndMotorSolenoid || s == AppState::RecordEndWaitArmHome;
}

static AppState state = AppState::IdleArmAtRest;
static unsigned long phaseStartMs = 0;

static bool liftUpTimerActive(unsigned long now) {
  return (now - phaseStartMs) < ARM_LIFT_TRAVEL_MS;
}

/** Timed UP: run up until timer expires, then de-energise. */
static void runLiftTimedUp(unsigned long now) {
  if (!liftUpTimerActive(now)) {
    armLiftHardwareOff();
    return;
  }
  armLiftUp();
}

/**
 * DOWN: de-energise immediately when DOWN limit is reached; otherwise run down until
 * timer expires (failsafe).
 */
static void runLiftTimedDown(unsigned long now, bool reachedDown) {
  if (reachedDown) {
    armLiftHardwareOff();
    return;
  }
  if ((now - phaseStartMs) >= ARM_LIFT_TRAVEL_MS) {
    armLiftHardwareOff();
    return;
  }
  armLiftDown();
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BTN_ARM_LIFT, INPUT_PULLUP);
  pinMode(PIN_BTN_REJECT, INPUT_PULLUP);
  pinMode(PIN_IN_ARM_IN, INPUT_PULLUP);
  pinMode(PIN_IN_ARM_LIFT_DOWN, INPUT_PULLUP);
  pinMode(PIN_IN_RECORD_END, INPUT_PULLUP);

  pinMode(PIN_LED_POWER, OUTPUT);
  pinMode(PIN_LED_ARM_LIFT, OUTPUT);
  pinMode(PIN_LED_REJECT, OUTPUT);

  pinMode(PIN_OUT_ENERGISE_LIFT, OUTPUT);
  pinMode(PIN_OUT_HBRIDGE_A, OUTPUT);
  pinMode(PIN_OUT_HBRIDGE_B, OUTPUT);
  pinMode(PIN_OUT_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_OUT_REJECT_SOLENOID, OUTPUT);

  armLiftHardwareOff();
  setMotorEnable(false);
  setRejectSolenoid(false);
  digitalWrite(PIN_LED_POWER, HIGH);
  digitalWrite(PIN_LED_ARM_LIFT, LOW);
  digitalWrite(PIN_LED_REJECT, LOW);

  state = AppState::IdleArmAtRest;
  phaseStartMs = 0;

  Serial.print(F("boot state="));
  Serial.println(stateName(state));
}

void loop() {
  static bool armLiftLatchStable = false;
  static bool armLiftLatchLastRaw = false;
  static unsigned long armLiftLatchLastChg = 0;
  static bool armLiftLatchPrevStable = false;

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

  static AppState prevLoggedState = AppState::IdleArmAtRest;

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

  const bool armInFall = !armInStable && armInPrevStable;
  armInPrevStable = armInStable;
  (void)armInFall;

  const bool rejectPressEdge = rejectStable && !rejectPrevStable;
  rejectPrevStable = rejectStable;

  const bool recordEndRise = recordEndStable && !recordEndPrevStable;
  recordEndPrevStable = recordEndStable;

  const bool latchRise = armLiftLatchOn && !armLiftLatchPrevStable;
  const bool latchFall = !armLiftLatchOn && armLiftLatchPrevStable;
  (void)latchFall;
  armLiftLatchPrevStable = armLiftLatchOn;

  const bool inRecordEndRecovery =
      (state == AppState::RecordEndLiftTiming ||
       state == AppState::RecordEndMotorSolenoid ||
       state == AppState::RecordEndWaitArmHome ||
       state == AppState::RecordEndWaitRecordClear);

  auto beginRecordEndLift = [&]() {
    setMotorEnable(false);
    setRejectSolenoid(false);
    phaseStartMs = now;
    state = AppState::RecordEndLiftTiming;
    armLiftUp();
  };

  auto beginRejectSequence = [&]() {
    setRejectSolenoid(false);
    phaseStartMs = now;
    state = AppState::RejectLiftTiming;
    armLiftUp();
  };

  if (recordEndRise) {
    beginRecordEndLift();
  }

  if (rejectPressEdge && !recordEndStable && !inRecordEndRecovery &&
      !inRejectIndicatorFlow(state)) {
    beginRejectSequence();
  }

  switch (state) {

  case AppState::IdleArmAtRest:
    setMotorEnable(false);
    if (recordEndStable) {
      beginRecordEndLift();
      break;
    }
    if (armInStable) {
      phaseStartMs = now;
      state = AppState::SecurityLiftUp;
      armLiftUp();
      break;
    }
    if (latchRise) {
      phaseStartMs = now;
      state = AppState::ManualLiftUpTiming;
      armLiftUp();
    } else if (!armLiftLatchOn && !reachedDown) {
      phaseStartMs = now;
      state = AppState::ManualLiftDown;
      armLiftDown();
    } else {
      armLiftHardwareOff();
    }
    break;

  case AppState::ManualLiftDown:
    setMotorEnable(false);
    if (recordEndStable) {
      armLiftHardwareOff();
      state = AppState::RecordEndLiftTiming;
      phaseStartMs = now;
      armLiftUp();
      break;
    }
    if (armInStable) {
      armLiftHardwareOff();
      phaseStartMs = now;
      state = AppState::SecurityLiftUp;
      armLiftUp();
      break;
    }
    if (rejectPressEdge) {
      armLiftHardwareOff();
      beginRejectSequence();
      break;
    }
    if (armLiftLatchOn) {
      phaseStartMs = now;
      state = AppState::ManualLiftUpTiming;
      armLiftUp();
      break;
    }
    runLiftTimedDown(now, reachedDown);
    if (!digitalRead(PIN_OUT_ENERGISE_LIFT)) {
      state = AppState::IdleArmAtRest;
    }
    break;

  case AppState::ManualLiftUpTiming:
    setMotorEnable(false);
    if (recordEndStable) {
      armLiftHardwareOff();
      state = AppState::RecordEndLiftTiming;
      phaseStartMs = now;
      armLiftUp();
      break;
    }
    if (armInStable) {
      armLiftHardwareOff();
      phaseStartMs = now;
      state = AppState::SecurityLiftUp;
      armLiftUp();
      break;
    }
    if (rejectPressEdge) {
      armLiftHardwareOff();
      beginRejectSequence();
      break;
    }
    runLiftTimedUp(now);
    if (!digitalRead(PIN_OUT_ENERGISE_LIFT)) {
      state = AppState::IdleArmAtRest;
    } else if (!armLiftLatchOn) {
      armLiftHardwareOff();
      phaseStartMs = now;
      state = AppState::ManualLiftDown;
      armLiftDown();
    }
    break;

  case AppState::SecurityLiftUp:
    setMotorEnable(false);
    if (recordEndStable) {
      phaseStartMs = now;
      state = AppState::RecordEndLiftTiming;
      armLiftUp();
      break;
    }
    if (!armInStable) {
      armLiftHardwareOff();
      state = AppState::IdleArmAtRest;
      break;
    }
    if (rejectPressEdge) {
      armLiftHardwareOff();
      beginRejectSequence();
      break;
    }
    runLiftTimedUp(now);
    if (!digitalRead(PIN_OUT_ENERGISE_LIFT)) {
      if (armInStable && !recordEndStable) {
        state = AppState::Playing;
        setMotorEnable(true);
      } else {
        state = AppState::IdleArmAtRest;
      }
    }
    break;

  case AppState::Playing:
    if (recordEndStable) {
      setMotorEnable(false);
      phaseStartMs = now;
      state = AppState::RecordEndLiftTiming;
      armLiftUp();
      break;
    }
    if (!armInStable) {
      setMotorEnable(false);
      armLiftHardwareOff();
      state = AppState::IdleArmAtRest;
      break;
    }
    if (rejectPressEdge) {
      setMotorEnable(false);
      beginRejectSequence();
      break;
    }
    setMotorEnable(true);
    break;

  case AppState::RecordEndLiftTiming:
    runLiftTimedUp(now);
    if (!digitalRead(PIN_OUT_ENERGISE_LIFT)) {
      if (recordEndStable) {
        armLiftHardwareOff();
        phaseStartMs = now;
        state = AppState::RecordEndMotorSolenoid;
        setMotorEnable(true);
        setRejectSolenoid(true);
      } else if (armInStable) {
        phaseStartMs = now;
        state = AppState::SecurityLiftUp;
        armLiftUp();
      } else {
        state = AppState::IdleArmAtRest;
      }
    }
    break;

  case AppState::RecordEndMotorSolenoid:
    armLiftHardwareOff();
    setMotorEnable(true);
    if ((now - phaseStartMs) < SOLENOID_ENGAGE_MS) {
      setRejectSolenoid(true);
    } else {
      setRejectSolenoid(false);
      state = AppState::RecordEndWaitArmHome;
    }
    break;

  case AppState::RecordEndWaitArmHome:
    armLiftHardwareOff();
    setRejectSolenoid(false);
    setMotorEnable(true);
    if (armIsParkedAtRest(armInStable)) {
      setMotorEnable(false);
      if (recordEndStable) {
        state = AppState::RecordEndWaitRecordClear;
      } else {
        state = AppState::IdleArmAtRest;
      }
    }
    break;

  case AppState::RecordEndWaitRecordClear:
    armLiftHardwareOff();
    setMotorEnable(false);
    setRejectSolenoid(false);
    if (!recordEndStable) {
      state = AppState::IdleArmAtRest;
    }
    break;

  case AppState::RejectLiftTiming:
    if (recordEndStable) {
      phaseStartMs = now;
      state = AppState::RecordEndLiftTiming;
      armLiftUp();
      break;
    }
    runLiftTimedUp(now);
    if (!digitalRead(PIN_OUT_ENERGISE_LIFT)) {
      phaseStartMs = now;
      state = AppState::RejectWaitStable;
    }
    break;

  case AppState::RejectWaitStable:
    armLiftHardwareOff();
    if (recordEndStable) {
      beginRecordEndLift();
      break;
    }
    if ((now - phaseStartMs) >= REJECT_STABLE_AFTER_LIFT_MS) {
      phaseStartMs = now;
      state = AppState::RejectSolenoidPulse;
      setRejectSolenoid(true);
    }
    break;

  case AppState::RejectSolenoidPulse:
    if (recordEndStable) {
      setRejectSolenoid(false);
      beginRecordEndLift();
      break;
    }
    if ((now - phaseStartMs) >= SOLENOID_ENGAGE_MS) {
      setRejectSolenoid(false);
      if (armInStable) {
        phaseStartMs = now;
        state = AppState::SecurityLiftUp;
        armLiftUp();
      } else {
        state = AppState::IdleArmAtRest;
      }
    }
    break;
  }

  if (liftIsEnergised()) {
    digitalWrite(PIN_LED_ARM_LIFT, ledFlashOn(now) ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_ARM_LIFT, LOW);
  }

  if (inRejectIndicatorFlow(state) || inRecordEndArmReturnIndicator(state)) {
    digitalWrite(PIN_LED_REJECT, ledFlashOn(now) ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_REJECT, LOW);
  }

  if (state != prevLoggedState) {
    Serial.print(now);
    Serial.print(F(" ms state -> "));
    Serial.println(stateName(state));
    prevLoggedState = state;
  }
}
