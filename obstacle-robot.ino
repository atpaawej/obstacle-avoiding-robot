// ============================================================================
//  Obstacle Avoiding Robot — v16 (stable cruise + double-ping debounce)
//  HW: Uno + HC-SR04(TRIG11,ECHO12) + Servo(3) + L298N(ENA5,7,8 ENB6,9,10) + 4WD + 3S 11.1V
//
//  FIXES APPLIED (v16):
//    1. PING_INTERVAL 50→80ms to avoid overlapping ultrasonic echoes (stable cruise).
//    2. CRUISE obstacle confirmation: require 2 consecutive pings with
//       (d > 4 && d <= STOP_CM) before AVOID_STOP. Single glitch no longer stops
//       the car. Clear ping (d > STOP_CM or 400) resets counter to 0.
//       STOP_CM stays 28, floor glitches ≤4 cm still ignored. PWM stays 190/180/190.
//    3. Avoidance stays clean: stop 300ms → reverse 400ms → scan L/R → pivot to
//       greater clearance → escape forward → CRUISE. TURN_MS stays 650ms solid pivot.
//    (v15 fixes retained: AVOID_DECIDE never picks front, 400-on-timeout, stamps.)
//
//  BEHAVIOR:
//    STARTUP: scan L/F/R → turn to best → cruise
//    CRUISE:  drive forward + ping every 80ms, debounce 2× before blocking
//    BLOCKED: stop → reverse → scan L/R → pivot LEFT/RIGHT 90° → escape → cruise
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
#define PING_INTERVAL  80    // ms between front pings during cruise (v16: 50→80 to avoid echo overlap)
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
uint8_t blockHits = 0;      // v16: consecutive blocked pings in CRUISE (need 2× to trigger)

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  sensor_init();
  motors_init();
  servo_init();
  delay(600);  // one-time servo power-up settle

  Serial.println(F("=== ROBOT v16 fixed ==="));

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

    // ── CRUISE: ping front, motor stays ON — v16 double-ping debounce ──
    case CRUISE: {
      if (now - lastPing < PING_INTERVAL) return;  // not time yet (80ms)
      lastPing = now;
      int d = sensor_ping();
      if (d == 0) d = CLEAR_CM;  // defensive: 0 = timeout → clear (sensor_ping already returns 400)
      Serial.print(F("front ")); Serial.println(d);
      // v16: require 2 consecutive pings in (4, STOP_CM] before blocking.
      // Single stray echo no longer interrupts cruise. Clear ping resets counter.
      if (d > 4 && d <= STOP_CM) {
        blockHits++;
        Serial.print(F("hit ")); Serial.print(blockHits); Serial.println(F("/2"));
        if (blockHits >= 2) {
          Serial.print(F("BLOCKED d=")); Serial.println(d);
          motors_stop();
          blockHits = 0;
          stateStart = now;
          state = AVOID_STOP;
        }
      } else {
        if (blockHits != 0) Serial.println(F("clear -> reset hits"));
        blockHits = 0;  // clear reading (d > STOP_CM or 400 or ≤4 glitch) → reset
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
        blockHits = 0;      // v16: fresh debounce window on re-enter CRUISE
        stateStart = now;
        state = CRUISE;
        lastPing = now;
      }
      break;
  }
}
