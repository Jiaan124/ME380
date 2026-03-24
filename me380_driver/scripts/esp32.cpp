/*
BOARD: Arduino Uno
Shield: CNC Shield V3
LIBS: AccelStepper.h, Servo.h
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
const int COMMAND_PERIOD_MS = 20;
const int TRAJ_BUFFER_POINTS = 8;

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
int trajPointCount = 0;
int trajRowsReceived = 0;
int trajRowsDispatched = 0;
bool isLoadingTrajectory = false;
bool isExecutingTrajectory = false;
bool finishedFlagSent = false;
unsigned long lastCommandDispatch = 0;

bool parseFloatLine(const String& line, float* values, int expectedCount);
void handleSerialInput();
void startTrajectoryLoad(int count);
void loadTrajectoryRow(const String& line);
void dispatchTrajectoryPoint(const TrajectoryPoint& p);
void moveSteppers(float dJ1, float dJ2, float dJ3, float dJ4,
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

  if (isExecutingTrajectory && trajBufferedCount > 0 &&
      millis() - lastCommandDispatch >= COMMAND_PERIOD_MS) {
    const TrajectoryPoint p = trajBuffer[trajHead];
    trajHead = (trajHead + 1) % TRAJ_BUFFER_POINTS;
    trajBufferedCount--;
    dispatchTrajectoryPoint(p);
    trajRowsDispatched++;
    lastCommandDispatch = millis();

    if (trajRowsReceived < trajPointCount && trajBufferedCount < TRAJ_BUFFER_POINTS) {
      Serial.println("NEXT");
    }
  }

  if (isExecutingTrajectory &&
      trajRowsDispatched >= trajPointCount &&
      trajBufferedCount == 0 &&
      isMotionDone() &&
      !finishedFlagSent) {
    isExecutingTrajectory = false;
    isLoadingTrajectory = false;
    finishedFlagSent = true;
    Serial.println("FINISHED");
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

  if (line.startsWith("LOAD ")) {
    int count = line.substring(5).toInt();
    startTrajectoryLoad(count);
    return;
  }

  if (isLoadingTrajectory || isExecutingTrajectory) {
    loadTrajectoryRow(line);
    return;
  }

  Serial.println("ERR: expected LOAD <N>");
}

void startTrajectoryLoad(int count) {
  if (count <= 0) {
    Serial.println("ERR: invalid N");
    return;
  }

  trajPointCount = count;
  trajRowsReceived = 0;
  trajRowsDispatched = 0;
  trajHead = 0;
  trajTail = 0;
  trajBufferedCount = 0;
  isLoadingTrajectory = true;
  isExecutingTrajectory = false;
  finishedFlagSent = false;
  Serial.println("READY");
  Serial.println("NEXT");
}

void loadTrajectoryRow(const String& line) {
  if (trajRowsReceived >= trajPointCount) {
    Serial.println("ERR: too many rows");
    return;
  }

  if (trajBufferedCount >= TRAJ_BUFFER_POINTS) {
    Serial.println("ERR: wait NEXT");
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

  trajBuffer[trajTail] = p;
  trajTail = (trajTail + 1) % TRAJ_BUFFER_POINTS;
  trajBufferedCount++;
  trajRowsReceived++;

  if (!isExecutingTrajectory) {
    isExecutingTrajectory = true;
    lastCommandDispatch = millis() - COMMAND_PERIOD_MS;
    Serial.println("EXEC");
  }

  if (trajRowsReceived >= trajPointCount) {
    isLoadingTrajectory = false;
  }
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