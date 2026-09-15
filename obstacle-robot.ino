// ============================================================================
//  Obstacle Avoiding Robot — Optimized Firmware (2WD FWD Mode + NewPing)
//  Hardware: Arduino Uno + HC-SR04 + SG90 Servo + L298N + 3S Li-ion (11.1V)
//
//  CONFIGURATION:
//    - Front 2 Wheels Driving (FWD), Rear 2 Wheels Idle (unplug rear motors from L298N)
//    - Sensor: NewPing library for glitch-free distance measurements
//    - Lighting Feedback: Visual status on Pin 13 LED (Boot, Cruise, Alert, Turn)
//
//  BEHAVIOR:
//    1. BOOT: 3-second safety countdown (Pin 13 LED double-blinking).
//    2. STARTUP CHECK: Pings front.
//       - If clear (> 35 cm): Starts cruise forward immediately.
//       - If blocked (<= 35 cm): Scans Left & Right, turns 90° towards
//         clearer side, then starts cruise.
//    3. CRUISE: Front wheels pull forward while simultaneously checking front
//       every 60 ms. Servo remains locked forward.
//    4. AVOIDANCE: Stop immediately -> 1 step back -> Scan Left & Right ->
//       Turn 90° towards clearer side -> Resume cruise.
// ============================================================================

#include <Servo.h>
#include <NewPing.h>

// ============================================================================
//  PIN ASSIGNMENTS
// ============================================================================
#define PIN_LED    13    // Builtin status & feedback LED
#define PIN_SERVO   3    // Servo signal pin
#define PIN_TRIG   11    // Ultrasonic Trigger
#define PIN_ECHO   12    // Ultrasonic Echo

#define PIN_ENA     5    // Left Motor Speed (PWM)
#define PIN_IN1     7    // Left Motor Direction 1
#define PIN_IN2     8    // Left Motor Direction 2
#define PIN_ENB     6    // Right Motor Speed (PWM)
#define PIN_IN3     9    // Right Motor Direction 1
#define PIN_IN4    10    // Right Motor Direction 2

// ============================================================================
//  CALIBRATION / TUNING CONSTANTS
// ============================================================================
#define STOP_DIST_CM     35    // Obstacle trigger distance (cm)
#define MAX_DIST_CM     250    // Max distance sensor will measure (cm)

// 2WD FWD speeds for 3S 11.1V battery (smooth, strong, low current):
#define SPEED_CRUISE    140    // Cruise PWM (0-255)
#define SPEED_TURN      135    // Pivot PWM (0-255)
#define SPEED_REV       130    // Reverse PWM (0-255)

#define TURN_90_MS      850    // Time to execute ~90 degree pivot (tuned for 2WD FWD)
#define REV_STEP_MS     300    // Duration of 1 step backwards (ms)
#define SCAN_SETTLE_MS  350    // Time for servo to arrive & settle at angle (ms)

// Servo angles
#define SERVO_LEFT      160    // Left scan angle (deg)
#define SERVO_RIGHT      25    // Right scan angle (deg)
#define SERVO_CENTER     90    // Center forward angle (deg)

// ============================================================================
//  GLOBAL MODULES
// ============================================================================
NewPing sonar(PIN_TRIG, PIN_ECHO, MAX_DIST_CM);
Servo scanServo;

// ============================================================================
//  LED FEEDBACK HELPERS
// ============================================================================
void led_on()         { digitalWrite(PIN_LED, HIGH); }
void led_off()        { digitalWrite(PIN_LED, LOW);  }

void led_alert_flash() {
  for (int i = 0; i < 3; i++) {
    led_on();  delay(50);
    led_off(); delay(50);
  }
}

// ============================================================================
//  DISTANCE MEASUREMENT (Using NewPing)
// ============================================================================
// Returns distance in cm. If timeout/no echo, returns 400 (clear path).
int read_distance() {
  delay(30);  // Brief ultrasonic settle
  int cm = sonar.ping_cm();
  if (cm <= 0 || cm > MAX_DIST_CM) return 400; // 0 means timeout = clear
  return cm;
}

// Median of 3 pings for static scans (ultra-stable)
int read_distance_median() {
  int a = read_distance();
  int b = read_distance();
  int c = read_distance();
  // Find median of 3 values
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

// ============================================================================
//  MOTOR MODULE (Soft-Ramp to eliminate inrush current spikes)
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
  // Gentle soft-ramp over ~80ms to protect 5V rail from voltage sag
  uint8_t start = (spd > 70) ? (spd - 50) : spd;
  for (uint8_t s = start; s < spd; s += 10) {
    analogWrite(PIN_ENA, s);
    analogWrite(PIN_ENB, s);
    delay(15);
  }
  analogWrite(PIN_ENA, spd);   analogWrite(PIN_ENB, spd);
}

void motors_reverse(uint8_t spd) {
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  uint8_t start = (spd > 70) ? (spd - 50) : spd;
  for (uint8_t s = start; s < spd; s += 10) {
    analogWrite(PIN_ENA, s);
    analogWrite(PIN_ENB, s);
    delay(15);
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
void servo_init() {
  scanServo.attach(PIN_SERVO);
  scanServo.write(SERVO_CENTER);
}

void servo_lookLeft()  { scanServo.write(SERVO_LEFT);   }
void servo_lookRight() { scanServo.write(SERVO_RIGHT);  }
void servo_center()    { scanServo.write(SERVO_CENTER); }

// ============================================================================
//  AVOIDANCE ROUTINE: Stop -> 1 Step Back -> Scan L/R -> Pivot 90°
// ============================================================================
void performAvoidance() {
  Serial.println(F("[AVOID] Obstacle spotted! Stopping immediately..."));
  motors_stop();
  led_alert_flash();   // Visual feedback: 3 rapid flashes
  delay(150);          // Allow robot momentum & power rail to stabilize

  Serial.println(F("[AVOID] Step backwards..."));
  motors_reverse(SPEED_REV);
  delay(REV_STEP_MS);
  motors_stop();
  delay(150);          // Voltage recovery pause before servo moves

  // Scan Left
  Serial.println(F("[AVOID] Scanning LEFT..."));
  led_on();
  servo_lookLeft();
  delay(SCAN_SETTLE_MS);
  int distL = read_distance_median();
  Serial.print(F("[AVOID] Left distance: ")); Serial.print(distL); Serial.println(F(" cm"));

  // Scan Right
  Serial.println(F("[AVOID] Scanning RIGHT..."));
  led_off();
  servo_lookRight();
  delay(SCAN_SETTLE_MS);
  int distR = read_distance_median();
  Serial.print(F("[AVOID] Right distance: ")); Serial.print(distR); Serial.println(F(" cm"));

  // Re-center servo
  servo_center();
  delay(200);

  // Decide clearer direction
  if (distL >= distR) {
    Serial.println(F("[AVOID] Turning LEFT 90 degrees"));
    led_on();
    motors_pivotLeft(SPEED_TURN);
  } else {
    Serial.println(F("[AVOID] Turning RIGHT 90 degrees"));
    led_on();
    motors_pivotRight(SPEED_TURN);
  }
  delay(TURN_90_MS);
  motors_stop();
  led_off();
  delay(100);

  Serial.println(F("[ROBOT] Resuming CRUISE forward!"));
  led_on();
  motors_forward(SPEED_CRUISE);
}

// ============================================================================
//  SETUP ROUTINE
// ============================================================================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);

  Serial.println(F("========================================"));
  Serial.println(F("  OBSTACLE AVOIDING ROBOT (2WD FWD)     "));
  Serial.println(F("========================================"));

  motors_init();
  servo_init();

  // 3-Second Safety Countdown & Brownout Reset Indicator
  for (int i = 3; i >= 1; i--) {
    Serial.print(F("[BOOT] Safety countdown: ")); Serial.print(i); Serial.println(F("..."));
    for (int b = 0; b < 2; b++) {
      led_on();  delay(120);
      led_off(); delay(120);
    }
    delay(400);
  }
  led_on();  // Solid ON indicates robot is active

  // Settle servo forward and ping front
  Serial.println(F("[STARTUP] Checking FRONT obstacle..."));
  servo_center();
  delay(300);

  int distF = read_distance_median();
  Serial.print(F("[STARTUP] Front reading: ")); Serial.print(distF); Serial.println(F(" cm"));

  if (distF <= STOP_DIST_CM && distF > 3) {
    Serial.println(F("[STARTUP] Front BLOCKED. Performing initial turn..."));
    performAvoidance();
  } else {
    Serial.println(F("[STARTUP] Front CLEAR. Starting cruise directly!"));
    motors_forward(SPEED_CRUISE);
  }
}

// ============================================================================
//  MAIN LOOP — Continuous Cruise & Simultaneous Sensing
// ============================================================================
void loop() {
  // Front ping while wheels continue rolling forward
  int d = read_distance();

  // Check if obstacle is detected within threshold
  if (d <= STOP_DIST_CM && d > 3) {
    performAvoidance();
  } else {
    // Front is clear -> keep cruising forward smoothly
    led_on();
  }

  delay(30);  // Small delay between pings (total cycle ~60ms)
}
