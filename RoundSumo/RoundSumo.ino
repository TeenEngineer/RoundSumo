// ---------- SENSORS ----------
#define IRL 34
#define IRR 35

// ultrasonic sensors:
#define TRIGL 4
#define ECHOL 16
#define TRIGR 5
#define ECHOR 17

// ---------- LINE SENSORS ----------
#define LFL 32
#define LFR 33
#define LBL 21
#define LBR 22
const bool LINE_DETECTED_LOW = false;

// ---------- MOTORS ----------
#define ENA 18
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENB 19

// ---------- START BUTTON ----------
#define BTN_START 13

// ---------- SPEEDS ----------
const int SPEED_ATTACK = 245;
const int SPEED_TURN = 185;
const int SPEED_CENTER = 190;
const int SPEED_RUSH = 255;
const int SPEED_ESCAPE = 210;
const int SPEED_ESCAPE_BIAS = 150;

// ---------- MOTOR DIRECTION CALIBRATION ----------
// Keep these at 1 if the side moves physically forward for a positive command.
// Set one side to -1 if that side is reversed.
const int LEFT_DRIVE_SIGN = 1;
const int RIGHT_DRIVE_SIGN = 1;

// ---------- ULTRASONIC ----------
const long FRONT_DETECT_CM = 65;
const long FRONT_CLOSE_CM = 35;
const long FRONT_MIN_CM = 2;
const long FRONT_NO_TARGET_CM = 80;
const unsigned long ULTRASONIC_GAP_MS = 20;
const unsigned long FRONT_MEMORY_MS = 200;
const unsigned long IR_MEMORY_MS = 250;
const unsigned long LINE_MEMORY_MS = 120;
const unsigned long FRONT_LOCK_MS = 280;

// ---------- TACTICAL / TIMING ----------
const unsigned long START_DELAY_MS = 5180;
const unsigned long ESCAPE_HOLD_MS = 600;
const unsigned long SEARCH_RESET_MS = 1600;
const unsigned long STARTUP_SETTLE_MS = 120;
const unsigned long SEARCH_KICK_MS = 90;
const int PWM_FREQ_HZ = 20000;
const unsigned long DEBUG_PRINT_MS = 150;
const unsigned long BRAKE_HOLD_MS = 80;

uint8_t searchDir = 0;  // 0 - right, 1 - left
bool searchDirLocked = false;  // true when set by IR/lock; prevents random override
bool robotOn = false;
bool waitingStart = false;
unsigned long startTimer = 0;
unsigned long robotStartedAt = 0;
uint8_t frontLeftHits = 0;
uint8_t frontRightHits = 0;

unsigned long searchStartTime = 0;
uint8_t searchState = 0;  // 0-turn, 1-pause, 2-fast turn back
unsigned long stateStartTime = 0;

unsigned long lastUltrasonicRead = 0;
bool readLeftUltrasonicNext = true;
long frontLeftDistance = FRONT_NO_TARGET_CM;
long frontRightDistance = FRONT_NO_TARGET_CM;
unsigned long lastFrontLeftSeen = 0;
unsigned long lastFrontRightSeen = 0;
unsigned long lastIrLeftSeen = 0;
unsigned long lastIrRightSeen = 0;
unsigned long lastLineFLSeen = 0;
unsigned long lastLineFRSeen = 0;
unsigned long lastLineBLSeen = 0;
unsigned long lastLineBRSeen = 0;

bool escapeActive = false;
unsigned long escapeUntil = 0;
int escapeLeftSpeed = 0;
int escapeRightSpeed = 0;

unsigned long lastDebugPrint = 0;
bool brakeRushActive = false;
unsigned long brakeRushUntil = 0;
bool frontLockActive = false;
unsigned long frontLockUntil = 0;
int frontLockDirection = 0;  // -1 left, 1 right, 0 centered/unknown

// ---------- MOTOR FUNCTIONS ----------

void motorsStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void driveRaw(int leftSpeed, int rightSpeed) {
  leftSpeed *= LEFT_DRIVE_SIGN;
  rightSpeed *= RIGHT_DRIVE_SIGN;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Positive = physical forward on this robot.
  digitalWrite(IN1, leftSpeed < 0);
  digitalWrite(IN2, leftSpeed > 0);
  digitalWrite(IN3, rightSpeed < 0);
  digitalWrite(IN4, rightSpeed > 0);

  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));
}

void motorsForward(int spd) {
  driveRaw(spd, spd);
}

void motorsBackward(int spd) {
  driveRaw(-spd, -spd);
}

void motorsTurnLeft(int spd) {
  driveRaw(-spd, spd);
}

void motorsTurnRight(int spd) {
  driveRaw(spd, -spd);
}

void motorsArcLeft(int leftSpd, int rightSpd) {
  driveRaw(leftSpd, rightSpd);
}

void motorsArcRight(int leftSpd, int rightSpd) {
  driveRaw(leftSpd, rightSpd);
}

// ---------- SENSOR HELPERS ----------

bool isSideEnemyDetected(uint8_t sensorPin) {
  return digitalRead(sensorPin) == LOW;
}

bool isLineDetected(uint8_t sensorPin) {
  int value = digitalRead(sensorPin);
  return LINE_DETECTED_LOW ? (value == LOW) : (value == HIGH);
}

long readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 10000);
  if (duration == 0) {
    return FRONT_NO_TARGET_CM;
  }

  long distance = (long)(duration * 0.0343f / 2.0f);
  if (distance <= 0 || distance > FRONT_NO_TARGET_CM) {
    return FRONT_NO_TARGET_CM;
  }

  return distance;
}

void updateFrontSensors() {
  if (millis() - lastUltrasonicRead < ULTRASONIC_GAP_MS) {
    return;
  }

  lastUltrasonicRead = millis();

  // Suppress reads during the first few ms after start so the very first
  // ultrasonic ping (which often returns junk) cannot phantom-lock the robot.
  if (robotStartedAt != 0 && millis() - robotStartedAt < STARTUP_SETTLE_MS) {
    return;
  }

  if (readLeftUltrasonicNext) {
    frontLeftDistance = readUltrasonicCm(TRIGL, ECHOL);
    if (isFrontDistanceDetected(frontLeftDistance)) {
      if (frontLeftHits < 2) frontLeftHits++;
      if (frontLeftHits >= 2) {
        lastFrontLeftSeen = millis();
        frontLockActive = true;
        frontLockUntil = millis() + FRONT_LOCK_MS;
        frontLockDirection = -1;
      }
    } else {
      frontLeftHits = 0;
    }
  } else {
    frontRightDistance = readUltrasonicCm(TRIGR, ECHOR);
    if (isFrontDistanceDetected(frontRightDistance)) {
      if (frontRightHits < 2) frontRightHits++;
      if (frontRightHits >= 2) {
        lastFrontRightSeen = millis();
        frontLockActive = true;
        frontLockUntil = millis() + FRONT_LOCK_MS;
        frontLockDirection = 1;
      }
    } else {
      frontRightHits = 0;
    }
  }

  readLeftUltrasonicNext = !readLeftUltrasonicNext;
}

void updateFrontLockState(bool oppFL, bool oppFR) {
  if (oppFL && oppFR) {
    frontLockActive = true;
    frontLockUntil = millis() + FRONT_LOCK_MS;
    frontLockDirection = 0;
    return;
  }

  // Hysteresis: when only one side is currently visible but the lock
  // was already centered, don't snap to that side immediately — only
  // commit to a side after the centered memory has fully decayed.
  if (oppFL) {
    frontLockActive = true;
    frontLockUntil = millis() + FRONT_LOCK_MS;
    if (frontLockDirection != 0) {
      frontLockDirection = -1;
    }
    return;
  }

  if (oppFR) {
    frontLockActive = true;
    frontLockUntil = millis() + FRONT_LOCK_MS;
    if (frontLockDirection != 0) {
      frontLockDirection = 1;
    }
    return;
  }

  if (frontLockActive && millis() >= frontLockUntil) {
    frontLockActive = false;
    frontLockDirection = 0;
  }
}

bool isFrontLeftDetected() {
  return lastFrontLeftSeen != 0 && (millis() - lastFrontLeftSeen) <= FRONT_MEMORY_MS;
}

bool isFrontRightDetected() {
  return lastFrontRightSeen != 0 && (millis() - lastFrontRightSeen) <= FRONT_MEMORY_MS;
}

bool isEnemyCentered() {
  return isFrontLeftDetected() && isFrontRightDetected();
}

bool isFrontDistanceDetected(long distanceCm) {
  return distanceCm >= FRONT_MIN_CM && distanceCm <= FRONT_DETECT_CM;
}

bool isIrLeftDetected() {
  return lastIrLeftSeen != 0 && (millis() - lastIrLeftSeen) <= IR_MEMORY_MS;
}

bool isIrRightDetected() {
  return lastIrRightSeen != 0 && (millis() - lastIrRightSeen) <= IR_MEMORY_MS;
}

bool isLineFLDetected() {
  return lastLineFLSeen != 0 && (millis() - lastLineFLSeen) <= LINE_MEMORY_MS;
}

bool isLineFRDetected() {
  return lastLineFRSeen != 0 && (millis() - lastLineFRSeen) <= LINE_MEMORY_MS;
}

bool isLineBLDetected() {
  return lastLineBLSeen != 0 && (millis() - lastLineBLSeen) <= LINE_MEMORY_MS;
}

bool isLineBRDetected() {
  return lastLineBRSeen != 0 && (millis() - lastLineBRSeen) <= LINE_MEMORY_MS;
}

void motorsHardBrake() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

// ---------- EDGE ESCAPE ----------

void startEscape(int leftSpeed, int rightSpeed) {
  escapeActive = true;
  escapeUntil = millis() + ESCAPE_HOLD_MS;
  escapeLeftSpeed = leftSpeed;
  escapeRightSpeed = rightSpeed;
  brakeRushActive = false;
}

bool handleEscape() {
  if (!escapeActive) {
    return false;
  }

  if (millis() >= escapeUntil) {
    escapeActive = false;
    return false;
  }

  driveRaw(escapeLeftSpeed, escapeRightSpeed);
  return true;
}

void startBrakeRush() {
  brakeRushActive = true;
  brakeRushUntil = millis() + BRAKE_HOLD_MS;
}

bool handleBrakeRush() {
  if (!brakeRushActive) {
    return false;
  }

  if (millis() < brakeRushUntil) {
    motorsHardBrake();
    return true;
  }

  brakeRushActive = false;
  motorsForward(SPEED_RUSH);
  return true;
}

void handleLineEscape(bool lineFL, bool lineFR, bool lineBL, bool lineBR) {
  // Allow re-trigger if a new line is detected on the OPPOSITE side
  // of the current escape direction (so we don't drive off the far edge).
  if (escapeActive) {
    bool escapingBackward = escapeLeftSpeed < 0 || escapeRightSpeed < 0;
    bool escapingForward = escapeLeftSpeed > 0 && escapeRightSpeed > 0;
    bool oppositeHit = (escapingBackward && (lineBL || lineBR)) ||
                       (escapingForward && (lineFL || lineFR));
    if (!oppositeHit) {
      return;
    }
  }

  if (lineFL || lineFR) {
    if (lineFL && lineFR) {
      startEscape(-SPEED_ESCAPE, -SPEED_ESCAPE);
    } else if (lineFL) {
      startEscape(-SPEED_ESCAPE, -SPEED_ESCAPE_BIAS);
    } else {
      startEscape(-SPEED_ESCAPE_BIAS, -SPEED_ESCAPE);
    }
    return;
  }

  if (lineBL || lineBR) {
    if (lineBL && lineBR) {
      startEscape(SPEED_ESCAPE, SPEED_ESCAPE);
    } else if (lineBL) {
      startEscape(SPEED_ESCAPE, SPEED_ESCAPE_BIAS);
    } else {
      startEscape(SPEED_ESCAPE_BIAS, SPEED_ESCAPE);
    }
  }
}

// ---------- SEARCH ----------

void resetSearchState() {
  searchStartTime = 0;
  searchDirLocked = false;
}

void aggressiveSearchInPlace() {
  unsigned long now = millis();

  if (searchStartTime == 0) {
    searchStartTime = now;
    if (!searchDirLocked) {
      searchDir = random(0, 2);
    }
  }

  // Kick: during the first SEARCH_KICK_MS of a fresh spin, drive the
  // outer side beyond steady-state with the inner side actively braked.
  // This breaks static friction faster than a pure pivot.
  bool kicking = (now - searchStartTime) < SEARCH_KICK_MS;

  if (searchDir == 0) {
    if (kicking) {
      // Right pivot kick: outer (left) full forward, inner (right) full reverse + brake
      driveRaw(255, -255);
    } else {
      motorsTurnRight(255);
    }
  } else {
    if (kicking) {
      driveRaw(-255, 255);
    } else {
      motorsTurnLeft(255);
    }
  }

  if (now - searchStartTime > SEARCH_RESET_MS) {
    searchStartTime = now;
    if (!searchDirLocked) {
      searchDir = !searchDir;
    }
  }
}

// ---------- ATTACK ----------

void smartAttack(bool oppL, bool oppFL, bool oppFR, bool oppR) {
  // oppFL / oppFR are now memory-based, so they stay stable across the
  // alternating ultrasonic read pattern. Use them directly to avoid
  // single-frame flapping into the wrong branch.
  bool oppFC = oppFL && oppFR;

  if (oppFC) {
    bool veryClose =
      (frontLeftDistance >= FRONT_MIN_CM && frontLeftDistance <= FRONT_CLOSE_CM) ||
      (frontRightDistance >= FRONT_MIN_CM && frontRightDistance <= FRONT_CLOSE_CM);
    motorsForward(veryClose ? SPEED_RUSH : SPEED_ATTACK);
    return;
  }

  // Only ONE ultrasonic sees the target — pivot HARD toward that side
  // until both ultrasonics confirm before we charge.
  if (oppFL) {
    searchDir = 1;
    searchDirLocked = true;
    motorsTurnLeft(255);
    return;
  }

  if (oppFR) {
    searchDir = 0;
    searchDirLocked = true;
    motorsTurnRight(255);
    return;
  }

  // IR-only detection: spin toward the side that sees the enemy NOW.
  // This must come before the frontLock fall-through so the side IRs
  // can override a stale "centered" lock.
  if (oppL && !oppR) {
    searchDir = 1;
    searchDirLocked = true;
    motorsTurnLeft(255);
    return;
  }
  if (oppR && !oppL) {
    searchDir = 0;
    searchDirLocked = true;
    motorsTurnRight(255);
    return;
  }

  if (frontLockActive) {
    if (frontLockDirection < 0) {
      searchDir = 1;
      searchDirLocked = true;
      motorsTurnLeft(255);
      return;
    }
    if (frontLockDirection > 0) {
      searchDir = 0;
      searchDirLocked = true;
      motorsTurnRight(255);
      return;
    }
    // Centered lock with no current detection on either side.
    // Inch forward only briefly; then fall through to search.
    if (oppL && oppR) {
      motorsForward(SPEED_ATTACK);
      return;
    }
    // No corroborating sensor: don't drive blind, search instead.
  }

  aggressiveSearchInPlace();
}

// ---------- START BUTTON ----------

void handleStartButton() {
  static bool lastState = HIGH;
  bool state = digitalRead(BTN_START);

  if (lastState == HIGH && state == LOW) {
    delay(30);
    if (digitalRead(BTN_START) == LOW) {
      if (!robotOn && !waitingStart) {
        waitingStart = true;
        startTimer = millis();
        searchDir = random(0, 2);
      } else if (robotOn) {
        robotOn = false;
        waitingStart = false;
        motorsStop();
        escapeActive = false;
        frontLockActive = false;
        frontLockDirection = 0;
      }
    }
  }

  lastState = state;

  if (waitingStart && millis() - startTimer >= START_DELAY_MS) {
    waitingStart = false;
    robotOn = true;
    resetSearchState();
    frontLockActive = false;
    frontLockDirection = 0;
    frontLeftHits = 0;
    frontRightHits = 0;
    frontLeftDistance = FRONT_NO_TARGET_CM;
    frontRightDistance = FRONT_NO_TARGET_CM;
    lastFrontLeftSeen = 0;
    lastFrontRightSeen = 0;
    lastIrLeftSeen = 0;
    lastIrRightSeen = 0;
    lastLineFLSeen = 0;
    lastLineFRSeen = 0;
    lastLineBLSeen = 0;
    lastLineBRSeen = 0;
    robotStartedAt = millis();
    // searchDir already randomized at button press; keep it.
    searchDirLocked = false;
  }
}

// ---------- DEBUG ----------

void printDebug(bool lineFL, bool lineFR, bool lineBL, bool lineBR, bool oppL, bool oppFL, bool oppFR, bool oppR) {
  if (millis() - lastDebugPrint < DEBUG_PRINT_MS) {
    return;
  }

  lastDebugPrint = millis();

  Serial.print("LFL:");
  Serial.print(digitalRead(LFL));
  Serial.print("(");
  Serial.print(lineFL ? "ON" : "off");
  Serial.print(") LFR:");
  Serial.print(digitalRead(LFR));
  Serial.print("(");
  Serial.print(lineFR ? "ON" : "off");
  Serial.print(") LBL:");
  Serial.print(digitalRead(LBL));
  Serial.print("(");
  Serial.print(lineBL ? "ON" : "off");
  Serial.print(") LBR:");
  Serial.print(digitalRead(LBR));
  Serial.print("(");
  Serial.print(lineBR ? "ON" : "off");
  Serial.print(") | IRL:");
  Serial.print(oppL ? "ON" : "off");
  Serial.print(" IRR:");
  Serial.print(oppR ? "ON" : "off");
  Serial.print(" | USL:");
  Serial.print(frontLeftDistance);
  Serial.print("cm(");
  Serial.print(oppFL ? "ON" : "off");
  Serial.print(") USR:");
  Serial.print(frontRightDistance);
  Serial.print("cm(");
  Serial.print(oppFR ? "ON" : "off");
  Serial.println(")");
}

// ---------- SETUP ----------

void setup() {
  Serial.begin(115200);

  pinMode(IRL, INPUT);
  pinMode(IRR, INPUT);

  pinMode(TRIGL, OUTPUT);
  pinMode(ECHOL, INPUT);
  pinMode(TRIGR, OUTPUT);
  pinMode(ECHOR, INPUT);

  pinMode(LFL, INPUT_PULLUP);
  pinMode(LFR, INPUT_PULLUP);
  pinMode(LBL, INPUT_PULLUP);
  pinMode(LBR, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Drive the L298N enable pins at 20kHz instead of the ESP32 default
  // (~1kHz). 1kHz is in the audio band, leaks switching losses, and
  // produces less average torque on a 4-motor / single-driver setup.
  analogWriteFrequency(ENA, PWM_FREQ_HZ);
  analogWriteFrequency(ENB, PWM_FREQ_HZ);

  pinMode(BTN_START, INPUT_PULLUP);

  motorsStop();
  randomSeed(micros());
}

// ---------- MAIN LOOP ----------

void loop() {
  handleStartButton();

  if (!robotOn) {
    motorsStop();
    return;
  }

  bool irLeftNow = isSideEnemyDetected(IRL);
  bool irRightNow = isSideEnemyDetected(IRR);
  bool lineFLNow = isLineDetected(LFL);
  bool lineFRNow = isLineDetected(LFR);
  bool lineBLNow = isLineDetected(LBL);
  bool lineBRNow = isLineDetected(LBR);

  unsigned long now = millis();

  if (irLeftNow) {
    lastIrLeftSeen = now;
    searchDir = 1;
    searchDirLocked = true;
  }

  if (irRightNow) {
    lastIrRightSeen = now;
    searchDir = 0;
    searchDirLocked = true;
  }

  if (lineFLNow) {
    lastLineFLSeen = now;
  }

  if (lineFRNow) {
    lastLineFRSeen = now;
  }

  if (lineBLNow) {
    lastLineBLSeen = now;
  }

  if (lineBRNow) {
    lastLineBRSeen = now;
  }

  // Use memory-based detections to feed the decision logic. This is
  // the main jitter killer: alternating ultrasonic reads + single-frame
  // IR dropouts would otherwise flap detection state every loop.
  bool oppL = isIrLeftDetected();
  bool oppR = isIrRightDetected();
  bool lineFL = isLineFLDetected();
  bool lineFR = isLineFRDetected();
  bool lineBL = isLineBLDetected();
  bool lineBR = isLineBRDetected();

  if (lineFL || lineFR || lineBL || lineBR) {
    handleLineEscape(lineFL, lineFR, lineBL, lineBR);
  }

  if (handleEscape()) {
    printDebug(lineFL, lineFR, lineBL, lineBR, oppL, false, false, oppR);
    return;
  }

  updateFrontSensors();

  bool oppFL = isFrontLeftDetected();
  bool oppFR = isFrontRightDetected();
  updateFrontLockState(oppFL, oppFR);

  printDebug(lineFL, lineFR, lineBL, lineBR, oppL, oppFL, oppFR, oppR);

  if (handleBrakeRush()) {
    return;
  }

  bool anyEnemy = oppL || oppFL || oppFR || oppR;

  if (!anyEnemy && !frontLockActive) {
    aggressiveSearchInPlace();
    return;
  }

  // Transitioning into engagement — reset spin timer once, but DON'T
  // wipe searchDirLocked: IR / front-lock direction must persist.
  if (searchStartTime != 0) {
    searchStartTime = 0;
  }
  smartAttack(oppL, oppFL, oppFR, oppR);
}
