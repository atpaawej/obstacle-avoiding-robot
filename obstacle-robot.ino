// ============================================================================
//  Obstacle Avoiding Robot — Overhauled Control Firmware (v18 - Stable)
//  Hardware: Arduino Uno + HC-SR04 (TRIG 11, ECHO 12) + SG90 Servo (Pin 3)
//            + L298N Dual H-Bridge (ENA 5, IN1 7, IN2 8, ENB 6, IN3 9, IN4 10)
//            + 4WD Chassis + 3S LiPo Battery
//
//  KEY FEATURES:
//    1. RESET INDICATOR & 3s COUNTDOWN: Onboard LED (Pin 13) blinks in a distinct
//       cadence for 3 seconds on power-up / reset. If this blink pattern occurs
//       while driving on the floor, it proves an electrical brownout reset.
//    2. FAST PING INTERVAL: Pings front every 60 ms (16+ times/sec) so obstacles
//       are spotted in real time at cruise speed.
//    3. SOFT MOTOR RAMP: 45ms PWM ramp prevents surge current from dropping
//       the 5V line and triggering brownout resets.
//    4. STARTUP CHECK: Pings front first. If clear (> 35 cm), cruises immediately.
//       If blocked, scans Left & Right, pivots 90°, and then cruises.
//    5. INSTANT STOP & AVOIDANCE: Immediate stop, 1 step reverse, side scan,
//       and 90° pivot towards clearer direction.
// ============================================================================

#include <Servo.h>

// ============================================================================
//  PIN ASSIGNMENTS
// ============================================================================
#define PIN_LED    13    // Onboard status & reset indicator LED
#define PIN_SERVO   3
#define PIN_TRIG   11
#define PIN_ECHO   12
#define PIN_ENA     5
#define PIN_IN1     7
#define PIN_IN2     8
#define PIN_ENB     6
#define PIN_IN3     9
#define PIN_IN4    10

// ============================================================================
//  CALIBRATION / TUNING CONSTANTS
// ============================================================================
#define STOP_DIST_CM     35    // Distance threshold to trigger obstacle avoidance (cm)
#define MIN_VALID_CM      3    // Ignore motor electrical noise spikes <= 3cm
#define CLEAR_CM        400    // Value returned on timeout / wide open path (cm)

#define SPEED_CRUISE    135    // Reduced for 3S 11.1V battery (plenty fast, 40% less current)
#define SPEED_TURN      130    // Pivot speed for turns (0-255 PWM)
#define SPEED_REV       130    // Reverse speed (0-255 PWM)

#define TURN_90_MS      950    // Time to execute ~90 degree pivot at 130 PWM
#define REV_STEP_MS     300    // Duration of 1 step backwards (ms)
#define PING_INTERVAL    60    // Time between pings while cruising (ms) (optimal 60ms)
#define SCAN_SETTLE_MS  350    // Time for servo to arrive & settle at angle (ms)

// Servo Angles: (If servo points right when it says LEFT, swap 160 and 25)
#define SERVO_LEFT      160    // Left scan angle (deg)
#define SERVO_RIGHT      25    // Right scan angle (deg)
#define SERVO_CENTER     90    // Center / Forward angle (deg)
#define PING_TIMEOUT  25000    // pulseIn timeout µs (~4.2m max)

// ============================================================================
//  SENSOR MODULE
// ============================================================================
void sensor_init() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}

// Single ultrasonic ping. Returns distance in cm.
// Returns CLEAR_CM (400) if no echo received (path is clear) or out of range.
int sensor_ping() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, PING_TIMEOUT);
  if (us == 0) return CLEAR_CM;

  int cm = (int)(us * 0.0343 / 2.0);
  if (cm <= 0 || cm > 400) return CLEAR_CM;
  return cm;
}

// Median of 5 pings with brief delay — ultra-reliable for static scans
int sensor_ping_median() {
  int readings[5], sorted[5];
  for (int i = 0; i < 5; i++) {
    readings[i] = sensor_ping();
    delay(25);
  }
  for (int i = 0; i < 5; i++) sorted[i] = readings[i];
  for (int i = 1; i < 5; i++) {
    int key = sorted[i], j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  return sorted[2];
}

// ============================================================================
//  MOTOR MODULE (L298N with Soft Ramp to prevent Brownout Resets)
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
  // Gentle 100ms soft ramp to eliminate inrush current spike that causes brownout
  uint8_t start = (spd > 70) ? (spd - 50) : spd;
  for (uint8_t s = start; s < spd; s += 10) {
    analogWrite(PIN_ENA, s);
    analogWrite(PIN_ENB, s);
    delay(20);
  }
  analogWrite(PIN_ENA, spd);   analogWrite(PIN_ENB, spd);
}

void motors_reverse(uint8_t spd) {
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  // Gentle 100ms soft ramp
  uint8_t start = (spd > 70) ? (spd - 50) : spd;
  for (uint8_t s = start; s < spd; s += 10) {
    analogWrite(PIN_ENA, s);
    analogWrite(PIN_ENB, s);
    delay(20);
  }
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
}

void servo_lookLeft()  { scanServo.write(SERVO_LEFT);   }
void servo_lookRight() { scanServo.write(SERVO_RIGHT);  }
void servo_center()    { scanServo.write(SERVO_CENTER); }

// ============================================================================
//  STATE MACHINE DEFINITION
// ============================================================================
enum RobotState {
  STATE_CRUISE,
  STATE_AVOID_STOP,
  STATE_AVOID_REVERSE,
  STATE_AVOID_SCAN_LEFT,
  STATE_AVOID_SCAN_RIGHT,
  STATE_AVOID_DECIDE,
  STATE_AVOID_PIVOT,
  STATE_AVOID_RESUME
};

RobotState state = STATE_CRUISE;
uint32_t stateStartTime = 0;
uint32_t lastPingTime   = 0;
int avoidDir = 0;              // -1 = Left, 1 = Right
int distL = 0, distR = 0;

// Helper: Scan Left and Right, decide direction, and pivot 90°
void scanSidesAndTurn() {
  Serial.println(F("[AVOID] Pointing LEFT (scanning)..."));
  servo_lookLeft();
  delay(SCAN_SETTLE_MS);
  distL = sensor_ping_median();
  Serial.print(F("[AVOID] Left distance: ")); Serial.print(distL); Serial.println(F(" cm"));

  Serial.println(F("[AVOID] Pointing RIGHT (scanning)..."));
  servo_lookRight();
  delay(SCAN_SETTLE_MS);
  distR = sensor_ping_median();
  Serial.print(F("[AVOID] Right distance: ")); Serial.print(distR); Serial.println(F(" cm"));

  servo_center();
  delay(250);

  if (distL >= distR) {
    Serial.println(F("[AVOID] Turning LEFT 90 degrees"));
    motors_pivotLeft(SPEED_TURN);
  } else {
    Serial.println(F("[AVOID] Turning RIGHT 90 degrees"));
    motors_pivotRight(SPEED_TURN);
  }
  delay(TURN_90_MS);
  motors_stop();
  delay(100);
}

// ============================================================================
//  SETUP ROUTINE
// ============================================================================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);

  // --------------------------------------------------------------------------
  // RESET INDICATOR & 3-SECOND SAFETY COUNTDOWN
  // If the Arduino resets while driving on the floor, you will see this
  // double-blink countdown start over.
  // --------------------------------------------------------------------------
  Serial.println(F("========================================"));
  Serial.println(F("  OBSTACLE AVOIDING ROBOT BOOTING UP    "));
  Serial.println(F("========================================"));

  sensor_init();
  motors_init();
  servo_init();

  for (int i = 3; i >= 1; i--) {
    Serial.print(F("[BOOT] Safety countdown: ")); Serial.print(i); Serial.println(F("..."));
    for (int b = 0; b < 2; b++) {
      digitalWrite(PIN_LED, HIGH); delay(120);
      digitalWrite(PIN_LED, LOW);  delay(120);
    }
    delay(400);
  }
  digitalWrite(PIN_LED, HIGH);  // Solid ON indicates robot is active!

  Serial.println(F("[STARTUP] Checking FRONT obstacle..."));
  servo_center();
  delay(300);

  int distF = sensor_ping_median();
  Serial.print(F("[STARTUP] Front reading: ")); Serial.print(distF); Serial.println(F(" cm"));

  // Check if obstacle is present at startup (30-35cm range)
  if (distF <= STOP_DIST_CM && distF > MIN_VALID_CM) {
    Serial.println(F("[STARTUP] Front is BLOCKED. Scanning sides to pick direction..."));
    scanSidesAndTurn();
  } else {
    Serial.println(F("[STARTUP] Front is CLEAR. Proceeding directly to cruise!"));
  }

  // Begin continuous cruise
  Serial.println(F("[ROBOT] -> Starting CRUISE forward!"));
  motors_forward(SPEED_CRUISE);
  state = STATE_CRUISE;
  stateStartTime = millis();
  lastPingTime   = millis();
}

// ============================================================================
//  MAIN LOOP — Non-Blocking Cruise & Instant Obstacle Avoidance
// ============================================================================
void loop() {
  uint32_t now = millis();

  switch (state) {

    // ── CRUISE: Drive forward while simultaneously sensing front ────────────
    case STATE_CRUISE: {
      if (now - lastPingTime >= PING_INTERVAL) {
        lastPingTime = now;
        int d = sensor_ping();

        // Check for obstacle in front within detection distance
        if (d <= STOP_DIST_CM && d > MIN_VALID_CM) {
          Serial.print(F("[ALERT] Obstacle in front at: "));
          Serial.print(d);
          Serial.println(F(" cm! STOPPING IMMEDIATELY"));
          
          motors_stop();               // Instant stop!
          stateStartTime = now;
          state = STATE_AVOID_STOP;
        }
      }
      break;
    }

    // ── AVOID_STOP: Pause to kill robot momentum & let 5V rail stabilize ───
    case STATE_AVOID_STOP:
      if (now - stateStartTime >= 150) {
        Serial.println(F("[AVOID] Step backwards..."));
        motors_reverse(SPEED_REV);
        stateStartTime = now;
        state = STATE_AVOID_REVERSE;
      }
      break;

    // ── AVOID_REVERSE: Reverse 1 step backwards ─────────────────────────────
    case STATE_AVOID_REVERSE:
      if (now - stateStartTime >= REV_STEP_MS) {
        motors_stop();
        delay(150);  // Allow power rail to recharge completely before servo moves
        Serial.println(F("[AVOID] Stopped. Looking LEFT..."));
        servo_lookLeft();
        stateStartTime = now;
        state = STATE_AVOID_SCAN_LEFT;
      }
      break;

    // ── AVOID_SCAN_LEFT: Wait for servo settle, measure left clearance ──────
    case STATE_AVOID_SCAN_LEFT:
      if (now - stateStartTime >= SCAN_SETTLE_MS) {
        distL = sensor_ping_median();
        Serial.print(F("[AVOID] Left distance: ")); Serial.print(distL); Serial.println(F(" cm"));
        Serial.println(F("[AVOID] Looking RIGHT..."));
        servo_lookRight();
        stateStartTime = now;
        state = STATE_AVOID_SCAN_RIGHT;
      }
      break;

    // ── AVOID_SCAN_RIGHT: Wait for servo settle, measure right clearance ────
    case STATE_AVOID_SCAN_RIGHT:
      if (now - stateStartTime >= SCAN_SETTLE_MS) {
        distR = sensor_ping_median();
        Serial.print(F("[AVOID] Right distance: ")); Serial.print(distR); Serial.println(F(" cm"));
        servo_center();
        stateStartTime = now;
        state = STATE_AVOID_DECIDE;
      }
      break;

    // ── AVOID_DECIDE: Re-center servo, pick direction with greater clearance ─
    case STATE_AVOID_DECIDE:
      if (now - stateStartTime >= 250) {  // Allow servo to return to center
        if (distL >= distR) {
          avoidDir = -1;  // Left
          Serial.println(F("[AVOID] Decision: Turn LEFT 90 degrees"));
          motors_pivotLeft(SPEED_TURN);
        } else {
          avoidDir = 1;   // Right
          Serial.println(F("[AVOID] Decision: Turn RIGHT 90 degrees"));
          motors_pivotRight(SPEED_TURN);
        }
        stateStartTime = now;
        state = STATE_AVOID_PIVOT;
      }
      break;

    // ── AVOID_PIVOT: Pivot 90° for TURN_90_MS ───────────────────────────────
    case STATE_AVOID_PIVOT:
      if (now - stateStartTime >= TURN_90_MS) {
        motors_stop();
        stateStartTime = now;
        state = STATE_AVOID_RESUME;
      }
      break;

    // ── AVOID_RESUME: Brief settle after pivot, then resume CRUISE forward ───
    case STATE_AVOID_RESUME:
      if (now - stateStartTime >= 100) {
        Serial.println(F("[ROBOT] -> Resuming CRUISE forward!"));
        motors_forward(SPEED_CRUISE);
        lastPingTime = now;
        state = STATE_CRUISE;
      }
      break;
  }
}
