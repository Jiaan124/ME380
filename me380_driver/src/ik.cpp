float DH_Params[6][4] = { //Theta, d, alpha, a
    [0]={[0]=0, [1]=0.244107, [2]=-90*PI/180, [3]=-0.062242},
    [1]={[0]=0, [1]=0, [2]=0, [3]=0.21575},
    [2]={[0]=0, [1]=0, [2]=90*PI/180, [3]=0},
    [3]={[0]=0, [1]=0.1632, [2]=-90*PI/180, [3]=0},
    [4]={[0]=0, [1]=0, [2]=90*PI/180, [3]=0},
    [5]={[0]=0, [1]=0.13878, [2]=0, [3]=0}
};

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

const float STEPS_PER_REV = 200.0 * 16.0;
const float GEAR_RATIO_J1 = 2.8;
const float GEAR_RATIO_J2 = 48.0;
const float GEAR_RATIO_J3 = 40.0;

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

int currentPwmA = PWM_CENTER;
int currentPwmB = PWM_CENTER;

int targetPwmA = PWM_CENTER;
int targetPwmB = PWM_CENTER;

unsigned long lastServoUpdate = 0;

const double openAngle = 155;
const double closeAngle = 100; //60

//////////////////////////////
// ===== SETUP =====
//////////////////////////////

void setup() {

  Serial.begin(9600);

  pinMode(STEPPER_ENABLE, OUTPUT);
  digitalWrite(STEPPER_ENABLE, LOW);   // Enable drivers

  stepper1.setMaxSpeed(3000);
  stepper1.setAcceleration(1500);

  stepper2.setMaxSpeed(3000);
  stepper2.setAcceleration(1500);

  stepper3.setMaxSpeed(3000);
  stepper3.setAcceleration(1500);

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

  stepper1.run();
  stepper2.run();
  stepper3.run();

  updateServos();

  if (Serial.available()) {

    String line = Serial.readStringUntil('\n');
    line.trim();

    float values[6];
    int index = 0;

    char *token = strtok((char*)line.c_str(), " ");

    while (token != NULL && index < 6) {
      values[index++] = atof(token);
      token = strtok(NULL, " ");
    }

    if (index == 6) {

      moveSteppers(values[0], values[1], values[2]);
      moveDifferential(values[3], values[4]);
      moveEndEffector(values[5] == 1);

      Serial.println("Move Commanded");
    } 
    else {
      Serial.println("Invalid command format");
    }
  }
}

//////////////////////////////
// ===== STEPPER MOVE =====
//////////////////////////////

void moveSteppers(float dJ1, float dJ2, float dJ3) {

  long steps1 = dJ1 * STEPS_PER_REV * GEAR_RATIO_J1 / 360.0;
  long steps2 = dJ2 * STEPS_PER_REV * GEAR_RATIO_J2 / 360.0;
  long steps3 = dJ3 * STEPS_PER_REV * GEAR_RATIO_J3 / 360.0;

  stepper1.move(steps1);
  stepper2.move(steps2);
  stepper3.move(steps3);
}

//////////////////////////////
// ===== DIFFERENTIAL =====
//////////////////////////////

void moveDifferential(double angle4, double angle5) {

  double angleA = gearReduction * (angle4 + angle5);
  double angleB = gearReduction * (-angle4 + angle5);

  angleA = constrain(angleA, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);
  angleB = constrain(angleB, -SERVO_RANGE_DEG, SERVO_RANGE_DEG);

  targetPwmA = PWM_CENTER + angleA * US_PER_DEG;
  targetPwmB = PWM_CENTER + angleB * US_PER_DEG;

  targetPwmA = constrain(targetPwmA, PWM_MIN, PWM_MAX);
  targetPwmB = constrain(targetPwmB, PWM_MIN, PWM_MAX);
}

//////////////////////////////
// ===== END EFFECTOR =====
//////////////////////////////

void moveEndEffector(bool isClosed) {
  if (isClosed)
    endEffector.write(closeAngle);
  else
    endEffector.write(openAngle);
}

//////////////////////////////
// ===== SERVO SMOOTHING =====
//////////////////////////////

void updateServos() {

  if (millis() - lastServoUpdate < UPDATE_PERIOD_MS)
    return;

  lastServoUpdate = millis();

  double stepUs = SERVO_SPEED_US_PER_SEC * (UPDATE_PERIOD_MS / 1000.0);

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