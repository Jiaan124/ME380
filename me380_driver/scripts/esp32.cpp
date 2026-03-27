/*
 * ME380 robot firmware — serial trajectory interface
 *
 * HARDWARE
 *   Board: Arduino Uno (see comments in code for CNC Shield V3 pin map.)
 *   Libraries: AccelStepper, Servo
 *
 * SERIAL
 *   - Configure baud in setup() with Serial.begin(...); host must match.
 *   - Messages are line-oriented: one command per line, terminated by '\n'.
 *   - Numbers may be separated by spaces, commas, or tabs (see parseFloatLine).
 *
 * SERIAL COMMAND PROTOCOL (streaming)
 *
 *   Host sends rows one at a time. Supported formats:
 *
 *     13-value row:
 *        [0..5]   Joint 1..6 absolute position fields (degrees), interpreted as:
 *                 - Joints 1..4: delta angle for this timestep (typically
 *                   one 20 ms sample at 50 Hz). Step commands accumulate
 *                   (AccelStepper::move) so bursts do not cancel in-flight motion.
 *                 - Joints 5..6: absolute joint angles (deg) for the differential
 *                   wrist; not accumulated across rows.
 *
 *        [6]      Gripper: passed to Servo.write (clamped 0..180).
 *
 *        [7..12]  Joint 1..6 velocity (deg/s). Used as motion rate hints:
 *     7-value row:
 *        [0..5]   Joint 1..6 absolute positions (deg)
 *        [6]      Gripper
 *        Velocity defaults to 30 deg/s for all joints.
 *
 *   Commands:
 *     ZERO        - Set internal commanded position state to zero for J1..J4.
 *
 *                 - Stepper speeds use abs(velocity); step direction follows
 *                   the sign of the delta, not the velocity sign.
 *                 - Differential servos use max(abs(vel5), abs(vel6)) for slew.
 *
 *   Board applies each received row immediately (commands may overlap).
 *
 *   When the mechanics reach target, board sends:
 *      FINISHED
 *
 *   FINISHED marks that the current commanded motion has settled.
 *
 * ERROR LINES (examples)
 *   ERR: row must have 7 or 13 numbers — bad row format.
 *
 * SIGN CONVENTIONS (software vs hardware)
 *   - Joint 2 and joint 4 stepper deltas are negated in moveSteppers() so
 *     positive commanded deltas match the intended physical direction.
 *   - Joint 6 (sixth axis in the row) is negated before the differential map
 *     so the composed sixth axis matches convention; both wrist servos are
 *     driven from the same moveDifferential() mix.
 *
 * STARTUP
 *   On reset, board may print a ready banner (e.g. "CNC Shield Robot Ready").
 */


 #include <AccelStepper.h>
 #include <Servo.h>
 
 //////////////////////////////
 // ===== CNC SHIELD PINS =====
 //////////////////////////////
 
 // X Axis → J1
 #define J1_STEP 2
 #define J1_DIR  5
 
 // Y Axis → J2
 #define J2_STEP 3
 #define J2_DIR  6
 
 // Z Axis → J3
 #define J3_STEP 4
 #define J3_DIR  7
 
 // A Axis → J4
 #define J4_STEP 12
 #define J4_DIR  13
 
 #define STEPPER_ENABLE 8
 
 // ===== SERVOS (via endstop S pins) =====
 #define SERVO_A_PIN 9    // X-
 #define SERVO_B_PIN 10   // Y-
 #define EE_PIN      11   // Z-
 
 //////////////////////////////
 // ===== STEPPERS =====
 //////////////////////////////
 
 AccelStepper stepper1(AccelStepper::DRIVER, J1_STEP, J1_DIR);
 AccelStepper stepper2(AccelStepper::DRIVER, J2_STEP, J2_DIR);
 AccelStepper stepper3(AccelStepper::DRIVER, J3_STEP, J3_DIR);
 AccelStepper stepper4(AccelStepper::DRIVER, J4_STEP, J4_DIR);
 
 
 
 const float STEPS_PER_REV = 200.0 * 16.0;
 const float GEAR_RATIO_J1 = 5;
 const float GEAR_RATIO_J2 = 58.5;
 const float GEAR_RATIO_J3 = 52.0;
 const float GEAR_RATIO_J4 = 4.0;
 
 
 //////////////////////////////
 // ===== SERVOS =====
 //////////////////////////////
 
 Servo servoA;
 Servo servoB;
 Servo endEffector;
 
 const double gearReduction = 3.0 / 4.0;
 
 const int PWM_CENTER = 1500;
 const int PWM_MIN = 500;
 const int PWM_MAX = 2500;
 
 const double SERVO_RANGE_DEG = 135.0;
 const double US_PER_DEG = 2000.0 / 270.0;
 
 const double SERVO_SPEED_US_PER_SEC = 400.0;
const float DEFAULT_COMMAND_VEL_DEG_S = 30.0f;
 const int UPDATE_PERIOD_MS = 20;
 
 int currentPwmA = PWM_CENTER;
 int currentPwmB = PWM_CENTER;
 
 int targetPwmA = PWM_CENTER;
 int targetPwmB = PWM_CENTER;
 
 unsigned long lastServoUpdate = 0;
 double servoSpeedUsPerSec = SERVO_SPEED_US_PER_SEC;
 
 const double openAngle = 155;
 const double closeAngle = 100; //60
 
 struct TrajectoryPoint {
  // J1..J6: absolute joint angles (deg).
  float absDeg[6];
   float velocityDeg[6];  // J1..J6 velocity (deg/s)
   float gripperPos;      // Gripper angle command
 };
 
 bool isExecutingTrajectory = false;
 bool finishedFlagSent = false;
// Internal commanded absolute state for steppers J1..J4 (deg).
float commandedAbsDeg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
 
 bool parseFloatLine(const String& line, float* values, int expectedCount);
 void handleSerialInput();
 void loadTrajectoryRow(const String& line);
bool handleTextCommand(const String& line);
void zeroPositionState();
 void dispatchTrajectoryPoint(const TrajectoryPoint& p);
void moveSteppersAbsolute(float aJ1, float aJ2, float aJ3, float aJ4,
                   float vJ1, float vJ2, float vJ3, float vJ4);
 void moveDifferential(double angle5, double angle6, double vel5, double vel6);
 void moveEndEffector(float gripperPos);
 bool isMotionDone();
 int stepToward(int current, int target, double step);
 
 //////////////////////////////
 // ===== SETUP =====
 //////////////////////////////
 
 void setup() {
 
   Serial.begin(115200);
 
   pinMode(STEPPER_ENABLE, OUTPUT);
   digitalWrite(STEPPER_ENABLE, LOW);   // Enable drivers
 
   stepper1.setMaxSpeed(4000);
 
   stepper2.setMaxSpeed(10000);

   stepper3.setMaxSpeed(10000);
 
   stepper4.setMaxSpeed(4000);

 
   servoA.attach(SERVO_A_PIN);
   servoB.attach(SERVO_B_PIN);
   endEffector.attach(EE_PIN);
 
   servoA.writeMicroseconds(PWM_CENTER);
   servoB.writeMicroseconds(PWM_CENTER);
   endEffector.write(openAngle);
 
   Serial.println("CNC Shield Robot Ready");
 }
 
 //////////////////////////////
 // ===== MAIN LOOP =====
 //////////////////////////////
 
 void loop() {
 
   stepper1.runSpeedToPosition();
   stepper2.runSpeedToPosition();
   stepper3.runSpeedToPosition();
   stepper4.runSpeedToPosition();
 
   updateServos();
   handleSerialInput();
 
   // Emit one FINISHED for this specific command when motion settles.
   if (isExecutingTrajectory && isMotionDone() && !finishedFlagSent) {
     finishedFlagSent = true;
     Serial.println("FINISHED");
     isExecutingTrajectory = false;
   }
 }
 
 //////////////////////////////
 // ===== STEPPER MOVE =====
 //////////////////////////////
 
void moveSteppersAbsolute(float aJ1, float aJ2, float aJ3, float aJ4,
                   float vJ1, float vJ2, float vJ3, float vJ4) {
  // Convert absolute commanded angles to incremental deltas for AccelStepper::move().
  float dJ1 = aJ1 - commandedAbsDeg[0];
  float dJ2 = aJ2 - commandedAbsDeg[1];
  float dJ3 = aJ3 - commandedAbsDeg[2];
  float dJ4 = aJ4 - commandedAbsDeg[3];

  commandedAbsDeg[0] = aJ1;
  commandedAbsDeg[1] = aJ2;
  commandedAbsDeg[2] = aJ3;
  commandedAbsDeg[3] = aJ4;
 
   // Positive direction reversed for joints 2 and 4 (hardware convention).
   dJ2 = -dJ2;
   dJ4 = -dJ4;
 
   long steps1 = dJ1 * STEPS_PER_REV * GEAR_RATIO_J1 / 360.0;
   long steps2 = dJ2 * STEPS_PER_REV * GEAR_RATIO_J2 / 360.0;
   long steps3 = dJ3 * STEPS_PER_REV * GEAR_RATIO_J3 / 360.0;
   long steps4 = dJ4 * STEPS_PER_REV * GEAR_RATIO_J4 / 360.0;
 
   float speed1 = abs(vJ1) * STEPS_PER_REV * GEAR_RATIO_J1 / 360.0;
   float speed2 = abs(vJ2) * STEPS_PER_REV * GEAR_RATIO_J2 / 360.0;
   float speed3 = abs(vJ3) * STEPS_PER_REV * GEAR_RATIO_J3 / 360.0;
   float speed4 = abs(vJ4) * STEPS_PER_REV * GEAR_RATIO_J4 / 360.0;
 
   // Accumulate on the existing target so 50 Hz packets do not overwrite
   // unfinished motion that is still being executed.
   stepper1.move(steps1);
   stepper2.move(steps2);
   stepper3.move(steps3);
   stepper4.move(steps4);
 
   stepper1.setSpeed(steps1 >= 0 ? max(1.0f, speed1) : -max(1.0f, speed1));
   stepper2.setSpeed(steps2 >= 0 ? max(1.0f, speed2) : -max(1.0f, speed2));
   stepper3.setSpeed(steps3 >= 0 ? max(1.0f, speed3) : -max(1.0f, speed3));
   stepper4.setSpeed(steps4 >= 0 ? max(1.0f, speed4) : -max(1.0f, speed4));
 }
 
 //////////////////////////////
 // ===== DIFFERENTIAL =====
 //////////////////////////////
 
 void moveDifferential(double angle5, double angle6, double vel5, double vel6) {
 
   double angleA = gearReduction * (angle5 + angle6);
   double angleB = gearReduction * (-angle5 + angle6);
 
   angleA = constrain(angleA, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);
   angleB = constrain(angleB, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);
 
   targetPwmA = PWM_CENTER + angleA * US_PER_DEG;
   targetPwmB = PWM_CENTER + angleB * US_PER_DEG;
 
   targetPwmA = constrain(targetPwmA, PWM_MIN, PWM_MAX);
   targetPwmB = constrain(targetPwmB, PWM_MIN, PWM_MAX);
 
   double velDegPerSec = max(abs(vel5), abs(vel6));
   servoSpeedUsPerSec = max(50.0, velDegPerSec * US_PER_DEG);
 }
 
 //////////////////////////////
 // ===== END EFFECTOR =====
 //////////////////////////////
 
 void moveEndEffector(float gripperPos) {
   endEffector.write(constrain(gripperPos, 0, 180));
 }
 
 //////////////////////////////
 // ===== SERVO SMOOTHING =====
 //////////////////////////////
 
 void updateServos() {
 
   if (millis() - lastServoUpdate < UPDATE_PERIOD_MS)
     return;
 
   lastServoUpdate = millis();
 
   double stepUs = servoSpeedUsPerSec * (UPDATE_PERIOD_MS / 1000.0);
 
   currentPwmA = stepToward(currentPwmA, targetPwmA, stepUs);
   currentPwmB = stepToward(currentPwmB, targetPwmB, stepUs);
 
   servoA.writeMicroseconds(currentPwmA);
   servoB.writeMicroseconds(currentPwmB);
 }
 
 int stepToward(int current, int target, double step) {
 
   if (abs(target - current) <= step)
     return target;
 
   return current + (target > current ? step : -step);
 }
 
 bool parseFloatLine(const String& line, float* values, int expectedCount) {
   int lineLen = line.length();
   if (lineLen <= 0 || lineLen >= 256) {
     return false;
   }
 
   char buffer[256];
   line.toCharArray(buffer, sizeof(buffer));
 
   int index = 0;
   char* token = strtok(buffer, " ,\t");
   while (token != NULL && index < expectedCount) {
     values[index++] = atof(token);
     token = strtok(NULL, " ,\t");
   }
 
   return index == expectedCount;
 }
 
 void handleSerialInput() {
   if (!Serial.available()) {
     return;
   }
 
   String line = Serial.readStringUntil('\n');
   line.trim();
   if (line.length() == 0) {
     return;
   }
  if (handleTextCommand(line)) {
    return;
  }
   loadTrajectoryRow(line);
 }

bool handleTextCommand(const String& line) {
  if (line == "ZERO") {
    zeroPositionState();
    Serial.println("OK: ZERO");
    return true;
  }
  return false;
}

void zeroPositionState() {
  commandedAbsDeg[0] = 0.0f;
  commandedAbsDeg[1] = 0.0f;
  commandedAbsDeg[2] = 0.0f;
  commandedAbsDeg[3] = 0.0f;
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
}
 
 void loadTrajectoryRow(const String& line) {
   float values[13];
  bool parsed13 = parseFloatLine(line, values, 13);
  bool parsed7 = false;
  if (!parsed13) {
    parsed7 = parseFloatLine(line, values, 7);
  }
  if (!parsed13 && !parsed7) {
    Serial.println("ERR: row must have 7 or 13 numbers");
     return;
   }
 
   TrajectoryPoint p;
  for (int i = 0; i < 6; i++) {
    p.absDeg[i] = values[i];
    p.velocityDeg[i] = parsed13 ? values[i + 7] : DEFAULT_COMMAND_VEL_DEG_S;
  }
  p.gripperPos = values[6];
 
 // Apply immediately. J1..J4 are absolute and converted to incremental motion
 // against internal commandedAbsDeg[].
  dispatchTrajectoryPoint(p);
  isExecutingTrajectory = true;
  finishedFlagSent = false;
 }
 
 void dispatchTrajectoryPoint(const TrajectoryPoint& p) {
 
  moveSteppersAbsolute(
    p.absDeg[0], p.absDeg[1], p.absDeg[2], p.absDeg[3],
     p.velocityDeg[0], p.velocityDeg[1], p.velocityDeg[2], p.velocityDeg[3]
   );
 
   // J5/J6: absolute positions. Joint 6 sign is flipped in the differential mix
   // (the composed sixth axis), not by inverting a single servo only.
  moveDifferential(p.absDeg[4], -p.absDeg[5], p.velocityDeg[4], -p.velocityDeg[5]);
   moveEndEffector(p.gripperPos);
 }
 
 bool isMotionDone() {
   return stepper1.distanceToGo() == 0 &&
          stepper2.distanceToGo() == 0 &&
          stepper3.distanceToGo() == 0 &&
          stepper4.distanceToGo() == 0 &&
          currentPwmA == targetPwmA &&
          currentPwmB == targetPwmB;
 }