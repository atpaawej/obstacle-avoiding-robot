// ============================================================================
//  Obstacle Avoiding Robot — v15 (stutter-loop + glitch fixes)
//  HW: Uno + HC-SR04(TRIG11,ECHO12) + Servo(3) + L298N(ENA5,7,8 ENB6,9,10) + 4WD + 3S 11.1V
//
//  FIXES APPLIED (v15):
//    1. AVOID_DECIDE NEVER chooses front: this state is only reached after BLOCKED,
//       so going straight would re-hit the obstacle (infinite stutter loop).
//       Strictly distL vs distR → -1 (left) or 1 (right), whichever has clearance.
//    2. CRUISE trigger is now (d > 4 && d <= STOP_CM): readings ≤ 4 cm are floor
//       reflection glitches and are ignored. STOP_CM raised 20 → 28 for 4WD
//       stopping distance. Timeout/no-echo still normalizes to 400 (clear).
//    3. AVOID_TURN cleaned: dead front shortcut removed (unreachable now), pivot
//       always runs the full TURN_MS (650ms) before forward motion resumes.
//    (v14 fixes retained: 400-on-timeout, PWM 190/180/190, stateStart stamps.)
//
//  BEHAVIOR:
//    STARTUP: scan L/F/R → turn to best → cruise
//    CRUISE:  drive forward + ping front every 50ms (nothing else)
//    BLOCKED: stop → reverse → scan L/R → pivot LEFT or RIGHT 90° → escape → cruise
// ============================================================================

#include <Servo.h>

// ============================================================================
//  PINS (change ONLY here for different wiring)
// ============================================================================
#define PIN_SERVO  3
#define PIN_TRIG  11
#define PIN_ECHO  12
#define PIN_ENA    5
#define PIN_IN1    7
#define PIN_IN2    8
#define PIN_ENB    6
#define PIN_IN3    9
#define PIN_IN4   10

// ============================================================================
//  TUNING
// ============================================================================
#define STOP_CM        28    // v15: trigger avoidance if 4 < distance <= this (stopping distance)
#define CLEAR_CM      400    // normalized value for timeout / no-echo = clear path
#define SPEED_CRUISE  190    // cruise PWM (raised 130→190 for 4WD static friction on tile)
#define SPEED_TURN    180    // pivot PWM (raised 150→180)
#define SPEED_REV     190    // reverse PWM (raised 130→190)
#define TURN_MS       650    // pivot time for ~90°
#define REV_MS        400    // reverse time
#define ESCAPE_MS     500    // forward escape after turn
#define SCAN_SETTLE   450    // servo settle at extremes
#define PING_INTERVAL  50    // ms between front pings during cruise
#define SERVO_LEFT   160
#define SERVO_RIGHT   25
#define SERVO_CENTER  90
#define PING_TIMEOUT 25000   // pulseIn timeout µs → ~425cm max

// ============================================================================
//  SENSOR MODULE — raw pulseIn, no library hiding timeout
// ============================================================================
void sensor_init() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

// Returns cm. Returns CLEAR_CM (400) on timeout (no echo = no obstacle in range).
// FIX #1: 0 (timeout/garbage) is treated as CLEAR path, never as blocked.
int sensor_ping() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, PING_TIMEOUT);
  if (us == 0) return CLEAR_CM;          // timeout = nothing within range → clear
  int cm = (int)(us * 0.0343 / 2.0);
  if (cm <= 0 || cm > 400) return CLEAR_CM; // garbage beyond sensor range → clear
  return cm;
}

// Median of 5 pings — accurate, use when stopped
int sensor_ping_median() {
  int readings[5], sorted[5];
  for (int i = 0; i < 5; i++) {
    readings[i] = sensor_ping();
    delay(30);
  }
  // insertion sort
  for (int i = 0; i < 5; i++) sorted[i] = readings[i];
  for (int i = 1; i < 5; i++) {
    int key = sorted[i], j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  return sorted[2]; // median
}

// ============================================================================
//  MOTOR MODULE
// ============================================================================
void motors_init() {
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  motors_stop();
}

void motors_stop() {
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);     analogWrite(PIN_ENB, 0);
}

void motors_forward(uint8_t spd) {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, spd);   analogWrite(PIN_ENB, spd);
}

void motors_reverse(uint8_t spd) {
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, spd);   analogWrite(PIN_ENB, spd);
}

void motors_pivotLeft(uint8_t spd) {
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, spd);   analogWrite(PIN_ENB, spd);
}

void motors_pivotRight(uint8_t spd) {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  analogWrite(PIN_ENA, spd);   analogWrite(PIN_ENB, spd);
}

// ============================================================================
//  SERVO MODULE
// ============================================================================
Servo scanServo;

void servo_init() {
  scanServo.attach(PIN_SERVO);
  scanServo.write(SERVO_CENTER);
  // non-blocking: just write position, let caller handle timing
}

void servo_lookLeft()  { scanServo.write(SERVO_LEFT);  }
void servo_lookRight() { scanServo.write(SERVO_RIGHT); }
void servo_center()    { scanServo.write(SERVO_CENTER); }

// ============================================================================
//  STATE MACHINE
// ============================================================================
enum State {
  CRUISE,
  AVOID_STOP, AVOID_REVERSE, AVOID_SETTLE,
  AVOID_SCAN_LEFT, AVOID_SCAN_RIGHT, AVOID_SCAN_FRONT,
  AVOID_DECIDE, AVOID_TURN, AVOID_ESCAPE
};
State state = CRUISE;

uint32_t stateStart = 0;    // when we entered current state
uint32_t lastPing = 0;      // last front ping time
int avoidDir = 0;           // -1 left, 1 right (DECIDE never picks 0/front — see v15 fix)
int distL = 0, distR = 0, distF = 0;

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  sensor_init();
  motors_init();
  servo_init();
  delay(600);  // one-time servo power-up settle

  Serial.println(F("=== ROBOT v15 fixed ==="));

  // Startup: scan L/F/R (sensor returns 400 = clear on timeout)
  servo_lookLeft();  delay(SCAN_SETTLE); distL = sensor_ping_median();
  servo_lookRight(); delay(SCAN_SETTLE); distR = sensor_ping_median();
  servo_center();    delay(300);         distF = sensor_ping_median();
  if (distL == 0) distL = CLEAR_CM;
  if (distR == 0) distR = CLEAR_CM;
  if (distF == 0) distF = CLEAR_CM;

  Serial.print(F("L:")); Serial.print(distL);
  Serial.print(F(" F:")); Serial.print(distF);
  Serial.print(F(" R:")); Serial.println(distR);

  // pick best — FIX #2: front clear (incl. 400 = timeout) means go straight
  if (distF >= STOP_CM) avoidDir = 0;          // front clear (distF == 400 also lands here)
  else if (distL >= distR) avoidDir = -1;
  else avoidDir = 1;

  if (avoidDir == -1) { motors_pivotLeft(SPEED_TURN); delay(TURN_MS); }
  else if (avoidDir == 1) { motors_pivotRight(SPEED_TURN); delay(TURN_MS); }
  motors_stop(); delay(150);

  motors_forward(SPEED_CRUISE);
  state = CRUISE;
  stateStart = millis();   // FIX #4: stamp entry into CRUISE
  lastPing = millis();
  Serial.println(F("-> CRUISE"));
}

// ============================================================================
//  LOOP — fully non-blocking state machine
// ============================================================================
void loop() {
  uint32_t now = millis();

  switch (state) {

    // ── CRUISE: ping front, motor stays ON ──────────────────────────────
    case CRUISE: {
      if (now - lastPing < PING_INTERVAL) return;  // not time yet
      lastPing = now;
      int d = sensor_ping();
      if (d == 0) d = CLEAR_CM;  // defensive: 0 = timeout → clear (sensor_ping already returns 400)
      Serial.print(F("front ")); Serial.println(d);
      // FIX v15: only a real echo in (4, STOP_CM] blocks. ≤4 cm = floor glitch, ignore.
      // 400 (timeout) can never satisfy d <= STOP_CM, so open space keeps driving.
      if (d > 4 && d <= STOP_CM) {
        Serial.print(F("BLOCKED d=")); Serial.println(d);
        motors_stop();
        stateStart = now;   // FIX #4: stamp exit from CRUISE
        state = AVOID_STOP;
      }
      // else: motor keeps running (motors_forward was called before entering CRUISE)
      break;
    }

    // ── AVOID: stop, then reverse ───────────────────────────────────────
    case AVOID_STOP:
      if (now - stateStart >= 300) {
        motors_reverse(SPEED_REV);
        stateStart = now;
        state = AVOID_REVERSE;
      }
      break;

    case AVOID_REVERSE:
      if (now - stateStart >= REV_MS) {
        motors_stop();
        stateStart = now;
        state = AVOID_SETTLE;
      }
      break;

    // ── wait for servo+robot to settle before scanning ──────────────────
    case AVOID_SETTLE:
      if (now - stateStart >= 300) {
        servo_lookLeft();
        stateStart = now;
        state = AVOID_SCAN_LEFT;
      }
      break;

    // ── scan left ───────────────────────────────────────────────────────
    case AVOID_SCAN_LEFT:
      if (now - stateStart >= SCAN_SETTLE) {
        distL = sensor_ping_median();
        if (distL == 0) distL = CLEAR_CM;  // defensive normalize
        Serial.print(F("L:")); Serial.println(distL);
        servo_lookRight();
        stateStart = now;
        state = AVOID_SCAN_RIGHT;
      }
      break;

    // ── scan right ──────────────────────────────────────────────────────
    case AVOID_SCAN_RIGHT:
      if (now - stateStart >= SCAN_SETTLE) {
        distR = sensor_ping_median();
        if (distR == 0) distR = CLEAR_CM;  // defensive normalize
        Serial.print(F("R:")); Serial.println(distR);
        servo_center();
        stateStart = now;
        state = AVOID_SCAN_FRONT;
      }
      break;

    // ── scan front ──────────────────────────────────────────────────────
    case AVOID_SCAN_FRONT:
      if (now - stateStart >= 300) {
        distF = sensor_ping_median();
        if (distF == 0) distF = CLEAR_CM;  // defensive normalize
        Serial.print(F("F:")); Serial.println(distF);
        stateStart = now;   // FIX #4: stamp entry into DECIDE
        state = AVOID_DECIDE;
      }
      break;

    // ── decide direction ────────────────────────────────────────────────
    case AVOID_DECIDE: {
      // FIX v15: this state is ONLY reached after BLOCKED, so front is never
      // an option — picking it would drive straight back into the obstacle
      // (infinite stutter loop). Strictly left vs right on clearance.
      if (distL >= distR) {
        avoidDir = -1; // left has more (or equal) clearance
      } else {
        avoidDir = 1;  // right has more clearance
      }

      Serial.print(F("DIR="));
      if (avoidDir == -1) Serial.println(F("LEFT"));
      else Serial.println(F("RIGHT"));

      stateStart = now;
      state = AVOID_TURN;
      break;
    }

    // ── non-blocking pivot — always completes full TURN_MS ────────────────
    case AVOID_TURN: {
      // start pivot on first entry
      static bool pivoting = false;
      if (!pivoting) {
        // v15: avoidDir is always -1 or 1 here (DECIDE never picks front),
        // so every avoidance ends with a solid full-duration pivot.
        if (avoidDir == -1) motors_pivotLeft(SPEED_TURN);
        else motors_pivotRight(SPEED_TURN);
        pivoting = true;
        stateStart = now;
      }
      if (now - stateStart >= TURN_MS) {
        motors_stop();
        pivoting = false;
        stateStart = now;
        state = AVOID_ESCAPE;
      }
      break;
    }

    // ── escape forward then resume cruise ───────────────────────────────
    case AVOID_ESCAPE:
      if (now - stateStart >= 150) {  // small settle after stop
        motors_forward(SPEED_CRUISE);
        stateStart = now;   // FIX #4: stamp exit from ESCAPE
        state = CRUISE;
        lastPing = now;
      }
      break;
  }
}
