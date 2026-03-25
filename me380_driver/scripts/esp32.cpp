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
 *   Host sends rows one at a time. Each row is exactly 13 floating-point
 *      values in this order:
 *
 *        [0..5]   Joint 1..6 "position" fields (degrees), interpreted as:
 *                 - Joints 1..4: delta angle for this timestep (typically
 *                   one 20 ms sample at 50 Hz). Step commands accumulate
 *                   (AccelStepper::move) so bursts do not cancel in-flight motion.
 *                 - Joints 5..6: absolute joint angles (deg) for the differential
 *                   wrist; not accumulated across rows.
 *
 *        [6..11]  Joint 1..6 velocity (deg/s). Used as motion rate hints:
 *                 - Stepper speeds use abs(velocity); step direction follows
 *                   the sign of the delta, not the velocity sign.
 *                 - Differential servos use max(abs(vel5), abs(vel6)) for slew.
 *
 *        [12]     Gripper: passed to Servo.write (clamped 0..180).
 *
 *   Board stores incoming rows in a ring buffer (TRAJ_BUFFER_POINTS) and
 *   executes one buffered row at a time (strictly sequential commands).
 *
 *   If a new row arrives while motion is still running, it is queued (unless
 *   the buffer is full). This allows pipelined command streaming.
 *
 *   After the queue drains and all mechanics reach target, board sends:
 *      FINISHED
 *
 *   FINISHED marks that all queued commands are done and motion has settled.
 *
 * ERROR LINES (examples)
 *   ERR: buffer full           — queue full; retry later.
 *   ERR: row must have 13 numbers — bad row format.
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
 const int UPDATE_PERIOD_MS = 20;
 const int TRAJ_BUFFER_POINTS = 8;
const int VEL_MODE_START_BUFFER = 5;
 
 int currentPwmA = PWM_CENTER;
 int currentPwmB = PWM_CENTER;
 
 int targetPwmA = PWM_CENTER;
 int targetPwmB = PWM_CENTER;
 
 unsigned long lastServoUpdate = 0;
 double servoSpeedUsPerSec = SERVO_SPEED_US_PER_SEC;
 
 const double openAngle = 155;
 const double closeAngle = 100; //60
 
 struct TrajectoryPoint {
   // J1..J4: delta per timestep (deg). J5..J6: absolute joint angle (deg).
   float deltaDeg[6];
   float velocityDeg[6];  // J1..J6 velocity (deg/s)
   float gripperPos;      // Gripper angle command
 };
 
 TrajectoryPoint trajBuffer[TRAJ_BUFFER_POINTS];
 int trajHead = 0;
 int trajTail = 0;
 int trajBufferedCount = 0;
 bool isExecutingTrajectory = false;
 bool finishedFlagSent = false;
 
 bool parseFloatLine(const String& line, float* values, int expectedCount);
 void handleSerialInput();
bool handleModeCommand(const String& line);
 void loadTrajectoryRow(const String& line);
void loadVelocityRow(const String& line);
 void dispatchTrajectoryPoint(const TrajectoryPoint& p);
 void moveSteppers(float dJ1, float dJ2, float dJ3, float dJ4,
                   float vJ1, float vJ2, float vJ3, float vJ4);
 void moveDifferential(double angle5, double angle6, double vel5, double vel6);
 void moveEndEffector(float gripperPos);
 bool isMotionDone();
 int stepToward(int current, int target, double step);
void enqueueTrajectoryPoint(const TrajectoryPoint& p);

bool velocityModeEnabled = false;
unsigned long velocityModeTimeMs = 25;  // default vel_time
double velocityModeJoint5Deg = 0.0;
double velocityModeJoint6Deg = 0.0;
 
 //////////////////////////////
 // ===== SETUP =====
 //////////////////////////////
 
 void setup() {
 
   Serial.begin(115200);
 
   pinMode(STEPPER_ENABLE, OUTPUT);
   digitalWrite(STEPPER_ENABLE, LOW);   // Enable drivers
 
   stepper1.setMaxSpeed(3000);
 
   stepper2.setMaxSpeed(3000);
 
   stepper3.setMaxSpeed(3000);
 
   stepper4.setMaxSpeed(3000);
 
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
 
  // Execute one queued command at a time.
  // In velocity mode, wait until a minimum queue depth exists before
  // starting from idle to absorb short serial hiccups.
  bool readyToStart = trajBufferedCount > 0;
  if (velocityModeEnabled && !isExecutingTrajectory) {
    readyToStart = trajBufferedCount >= VEL_MODE_START_BUFFER;
  }
  if (!isExecutingTrajectory && readyToStart) {
     const TrajectoryPoint p = trajBuffer[trajHead];
     trajHead = (trajHead + 1) % TRAJ_BUFFER_POINTS;
     trajBufferedCount--;
     dispatchTrajectoryPoint(p);
     isExecutingTrajectory = true;
     finishedFlagSent = false;
   }
 
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
 
 void moveSteppers(float dJ1, float dJ2, float dJ3, float dJ4,
                   float vJ1, float vJ2, float vJ3, float vJ4) {
 
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
  if (handleModeCommand(line)) {
    return;
  }
  if (velocityModeEnabled) {
    loadVelocityRow(line);
  } else {
    loadTrajectoryRow(line);
  }
 }
 
bool handleModeCommand(const String& line) {
  int lineLen = line.length();
  if (lineLen <= 0 || lineLen >= 256) {
    return false;
  }

  char buffer[256];
  line.toCharArray(buffer, sizeof(buffer));

  char* token0 = strtok(buffer, " ,\t");
  if (token0 == NULL || String(token0) != "MODE") {
    return false;
  }

  char* token1 = strtok(NULL, " ,\t");
  if (token1 == NULL) {
    Serial.println("ERR: mode command format");
    return true;
  }

  String modeName(token1);
  if (modeName == "TRAJ") {
    velocityModeEnabled = false;
    Serial.println("MODE: TRAJ");
    return true;
  }

  if (modeName != "VEL") {
    Serial.println("ERR: mode must be TRAJ or VEL");
    return true;
  }

  char* token2 = strtok(NULL, " ,\t");
  if (token2 != NULL) {
    unsigned long parsed = (unsigned long)atol(token2);
    if (parsed == 0) {
      Serial.println("ERR: vel_time must be > 0 ms");
      return true;
    }
    velocityModeTimeMs = parsed;
  }

  velocityModeEnabled = true;
  Serial.print("MODE: VEL ");
  Serial.print(velocityModeTimeMs);
  Serial.println("ms");
  return true;
}

void enqueueTrajectoryPoint(const TrajectoryPoint& p) {
  trajBuffer[trajTail] = p;
  trajTail = (trajTail + 1) % TRAJ_BUFFER_POINTS;
  trajBufferedCount++;

  if (!isExecutingTrajectory) {
    finishedFlagSent = false;
  }
}

 void loadTrajectoryRow(const String& line) {
   if (trajBufferedCount >= TRAJ_BUFFER_POINTS) {
     Serial.println("ERR: buffer full");
     return;
   }
 
   float values[13];
   if (!parseFloatLine(line, values, 13)) {
     Serial.println("ERR: row must have 13 numbers");
     return;
   }
 
   TrajectoryPoint p;
   for (int i = 0; i < 6; i++) {
     p.deltaDeg[i] = values[i];
     p.velocityDeg[i] = values[i + 6];
   }
   p.gripperPos = values[12];
 
  enqueueTrajectoryPoint(p);
}

void loadVelocityRow(const String& line) {
  if (trajBufferedCount >= TRAJ_BUFFER_POINTS) {
    Serial.println("ERR: buffer full");
    return;
  }

  float values[7];
  if (!parseFloatLine(line, values, 7)) {
    Serial.println("ERR: velocity row must have 7 numbers");
    return;
  }

  const float dtSec = velocityModeTimeMs / 1000.0f;
  TrajectoryPoint p;

  // Velocity mode row format:
  // [0..5] = J1..J6 velocity (deg/s), [6] = gripper command.
  for (int i = 0; i < 4; i++) {
    p.velocityDeg[i] = values[i];
    p.deltaDeg[i] = values[i] * dtSec;
  }

  p.velocityDeg[4] = values[4];
  p.velocityDeg[5] = values[5];
  velocityModeJoint5Deg += values[4] * dtSec;
  velocityModeJoint6Deg += values[5] * dtSec;
  p.deltaDeg[4] = velocityModeJoint5Deg;
  p.deltaDeg[5] = velocityModeJoint6Deg;

  p.gripperPos = values[6];
  enqueueTrajectoryPoint(p);
 }
 
 void dispatchTrajectoryPoint(const TrajectoryPoint& p) {
 
   moveSteppers(
     p.deltaDeg[0], p.deltaDeg[1], p.deltaDeg[2], p.deltaDeg[3],
     p.velocityDeg[0], p.velocityDeg[1], p.velocityDeg[2], p.velocityDeg[3]
   );
 
   // J5/J6: absolute positions. Joint 6 sign is flipped in the differential mix
   // (the composed sixth axis), not by inverting a single servo only.
   moveDifferential(p.deltaDeg[4], -p.deltaDeg[5], p.velocityDeg[4], -p.velocityDeg[5]);
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