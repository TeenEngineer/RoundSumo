// ===== MOTOR PINS =====
#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25
#define ENA 18
#define ENB 19

// ===== BUTTON =====
#define BTN 13

// ===== ULTRASONIC =====
#define TRIG1 4
#define ECHO1 16

// ===== IR =====
#define IR1 34
#define IR2 35

// ===== LINE =====
#define L1 32
#define L2 33
#define L3 21
#define L4 22

// ===== STATE =====
bool started = false;
unsigned long startTime = 0;

// ===== SPEED SETTINGS =====
int baseSpeed = 160;
int attackSpeed = 255;
int turnFast = 200;
int turnSlow = 80;

// ===== TIMERS =====
unsigned long lastUS = 0;
long distance = 999;

// ===== MOTOR CONTROL =====
void setMotor(int left, int right) {
  // direction
  digitalWrite(IN1, left > 0);
  digitalWrite(IN2, left < 0);

  digitalWrite(IN3, right > 0);
  digitalWrite(IN4, right < 0);

  // speed
  analogWrite(ENA, abs(left));
  analogWrite(ENB, abs(right));
}

void stopMotors() {
  setMotor(0, 0);
}

// ===== ULTRASONIC =====
long readUS() {
  digitalWrite(TRIG1, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);

  long duration = pulseIn(ECHO1, HIGH, 20000);
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);

  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(L4, INPUT);

  stopMotors();
}

void loop() {

  // ===== START BUTTON =====
  if (!started && digitalRead(BTN) == LOW) {
    startTime = millis();
    started = true;
  }

  if (!started) return;

  // ===== 5s DELAY =====
  if (millis() - startTime < 5000) {
    stopMotors();
    return;
  }

  // ===== READ LINE (active LOW assumed) =====
  bool l1 = (digitalRead(L1) == LOW);
  bool l2 = (digitalRead(L2) == LOW);
  bool l3 = (digitalRead(L3) == LOW);
  bool l4 = (digitalRead(L4) == LOW);

  // ===== EDGE PRIORITY =====
  if (l1 || l2 || l3 || l4) {
    // quick escape
    setMotor(-200, -200);
    delay(120);

    // turn away
    setMotor(-200, 200);
    delay(200);
    return;
  }

  // ===== READ IR (ACTIVE LOW) =====
  bool ir1 = (digitalRead(IR1) == LOW);
  bool ir2 = (digitalRead(IR2) == LOW);

  // ===== ULTRASONIC UPDATE =====
  if (millis() - lastUS > 100) {
    lastUS = millis();
    distance = readUS();
  }

  // ===== ATTACK LOGIC =====

  // FRONT DETECTED
  if (ir1 && ir2) {
    setMotor(attackSpeed, attackSpeed);
    return;
  }

  // LEFT DETECTED → curve left
  if (ir1) {
    setMotor(turnSlow, turnFast);
    return;
  }

  // RIGHT DETECTED → curve right
  if (ir2) {
    setMotor(turnFast, turnSlow);
    return;
  }

  // ULTRASONIC fallback
  if (distance > 0 && distance < 50) {
    setMotor(baseSpeed, baseSpeed);
    return;
  }

  // ===== SEARCH MODE =====
  // smooth rotation (no stutter)
  setMotor(-120, 120);
}