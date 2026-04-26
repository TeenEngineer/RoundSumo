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
const long FRONT_DETECT_CM = 110;
const long FRONT_CLOSE_CM = 35;
const long FRONT_NO_TARGET_CM = 250;
const unsigned long ULTRASONIC_GAP_MS = 28;
const unsigned long FRONT_MEMORY_MS = 220;
const unsigned long IR_MEMORY_MS = 250;
const unsigned long LINE_MEMORY_MS = 120;

// ---------- TACTICAL / TIMING ----------
const unsigned long START_DELAY_MS = 3000;
const unsigned long ESCAPE_HOLD_MS = 3000;
const unsigned long SEARCH_TURN_MS = 600;
const unsigned long SEARCH_PAUSE_MS = 50;
const unsigned long SEARCH_FAST_TURN_MS = 150;
const unsigned long SEARCH_RESET_MS = 10000;
const unsigned long DEBUG_PRINT_MS = 150;

uint8_t searchDir = 0;  // 0 - right, 1 - left
bool robotOn = false;
bool waitingStart = false;
unsigned long startTimer = 0;

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

  if (readLeftUltrasonicNext) {
    frontLeftDistance = readUltrasonicCm(TRIGL, ECHOL);
    if (frontLeftDistance <= FRONT_DETECT_CM) {
      lastFrontLeftSeen = millis();
    }
  } else {
    frontRightDistance = readUltrasonicCm(TRIGR, ECHOR);
    if (frontRightDistance <= FRONT_DETECT_CM) {
      lastFrontRightSeen = millis();
    }
  }

  readLeftUltrasonicNext = !readLeftUltrasonicNext;
}

bool isFrontLeftDetected() {
  return (millis() - lastFrontLeftSeen) <= FRONT_MEMORY_MS;
}

bool isFrontRightDetected() {
  return (millis() - lastFrontRightSeen) <= FRONT_MEMORY_MS;
}

bool isEnemyCentered() {
  return isFrontLeftDetected() && isFrontRightDetected();
}

bool isIrLeftDetected() {
  return (millis() - lastIrLeftSeen) <= IR_MEMORY_MS;
}

bool isIrRightDetected() {
  return (millis() - lastIrRightSeen) <= IR_MEMORY_MS;
}

bool isLineFLDetected() {
  return (millis() - lastLineFLSeen) <= LINE_MEMORY_MS;
}

bool isLineFRDetected() {
  return (millis() - lastLineFRSeen) <= LINE_MEMORY_MS;
}

bool isLineBLDetected() {
  return (millis() - lastLineBLSeen) <= LINE_MEMORY_MS;
}

bool isLineBRDetected() {
  return (millis() - lastLineBRSeen) <= LINE_MEMORY_MS;
}

// ---------- EDGE ESCAPE ----------

void startEscape(int leftSpeed, int rightSpeed) {
  escapeActive = true;
  escapeUntil = millis() + ESCAPE_HOLD_MS;
  escapeLeftSpeed = leftSpeed;
  escapeRightSpeed = rightSpeed;
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

void handleLineEscape(bool lineFL, bool lineFR, bool lineBL, bool lineBR) {
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

  if (lineBL) {
    startEscape(SPEED_ESCAPE, SPEED_ESCAPE_BIAS);
    return;
  }

  if (lineBR) {
    startEscape(SPEED_ESCAPE_BIAS, SPEED_ESCAPE);
  }
}

// ---------- SEARCH ----------

void resetSearchState() {
  searchStartTime = 0;
  searchState = 0;
  stateStartTime = 0;
}

void aggressiveSearchInPlace() {
  unsigned long now = millis();

  if (searchStartTime == 0) {
    searchStartTime = now;
    stateStartTime = now;
    searchState = 0;
  }

  switch (searchState) {
    case 0:
      if (searchDir == 0) {
        motorsTurnRight(SPEED_TURN);
      } else {
        motorsTurnLeft(SPEED_TURN);
      }

      if (now - stateStartTime > SEARCH_TURN_MS) {
        motorsStop();
        searchState = 1;
        stateStartTime = now;
      }
      break;

    case 1:
      if (now - stateStartTime > SEARCH_PAUSE_MS) {
        searchState = 2;
        stateStartTime = now;
      }
      break;

    case 2:
      if (searchDir == 0) {
        motorsTurnLeft(min(255, SPEED_TURN + 30));
      } else {
        motorsTurnRight(min(255, SPEED_TURN + 30));
      }

      if (now - stateStartTime > SEARCH_FAST_TURN_MS) {
        motorsStop();
        searchState = 0;
        stateStartTime = now;
        searchDir = !searchDir;
      }
      break;
  }

  if (now - searchStartTime > SEARCH_RESET_MS) {
    searchStartTime = now;
    if (random(0, 100) < 30) {
      searchDir = !searchDir;
      searchState = 0;
      stateStartTime = millis();
    }
  }
}

// ---------- ATTACK ----------

void smartAttack(bool oppL, bool oppFL, bool oppFR, bool oppR) {
  bool oppFC = oppFL && oppFR;

  if (oppL && !oppFL && !oppFR) {
    motorsTurnLeft(255);
    return;
  }

  if (oppR && !oppFL && !oppFR) {
    motorsTurnRight(255);
    return;
  }

  if (oppFC) {
    if (frontLeftDistance <= FRONT_CLOSE_CM || frontRightDistance <= FRONT_CLOSE_CM) {
      motorsForward(SPEED_RUSH);
    } else {
      motorsForward(SPEED_ATTACK);
    }
    return;
  }

  if (oppFL) {
    motorsArcLeft(SPEED_CENTER - 40, SPEED_CENTER + 35);
    return;
  }

  if (oppFR) {
    motorsArcRight(SPEED_CENTER + 35, SPEED_CENTER - 40);
    return;
  }

  if (oppL) {
    motorsArcLeft(-SPEED_TURN, SPEED_TURN);
    return;
  }

  if (oppR) {
    motorsArcRight(SPEED_TURN, -SPEED_TURN);
    return;
  }

  motorsStop();
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
      }
    }
  }

  lastState = state;

  if (waitingStart && millis() - startTimer >= START_DELAY_MS) {
    waitingStart = false;
    robotOn = true;
    resetSearchState();
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
  }

  if (irRightNow) {
    lastIrRightSeen = now;
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

  bool oppL = isIrLeftDetected();
  bool oppR = isIrRightDetected();
  bool lineFL = isLineFLDetected();
  bool lineFR = isLineFRDetected();
  bool lineBL = isLineBLDetected();
  bool lineBR = isLineBRDetected();

  if (!escapeActive && (lineFL || lineFR || lineBL || lineBR)) {
    handleLineEscape(lineFL, lineFR, lineBL, lineBR);
  }

  if (handleEscape()) {
    printDebug(lineFL, lineFR, lineBL, lineBR, oppL, false, false, oppR);
    return;
  }

  // Skip ultrasonic polling on loops where the side IR already sees something.
  if (!(oppL || oppR)) {
    updateFrontSensors();
  }

  bool oppFL = isFrontLeftDetected();
  bool oppFR = isFrontRightDetected();

  printDebug(lineFL, lineFR, lineBL, lineBR, oppL, oppFL, oppFR, oppR);

  bool anyEnemy = oppL || oppFL || oppFR || oppR;

  if (anyEnemy) {
    resetSearchState();
    smartAttack(oppL, oppFL, oppFR, oppR);
  } else {
    aggressiveSearchInPlace();
  }
}
