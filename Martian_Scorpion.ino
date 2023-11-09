#include <NewPing.h>
#include <PID_v1.h>

/*
 *         [Մարսագնացի դիմացը]
 *              ------
 *             |      |
 *     M6 -> |          | <- M1
 *    (ENA)  |          |   (END)
 * (IN1,IN2) |          | (IN7,IN8)
 *             |      |
 *             --------
 *             |      |
 *     M5 -> |          |  <- M2
 *  (ENB)    |          |    (ENC)
 * (IN3,IN4) |          |   (IN4,IN5)
 *             |      |
 *             --------
 *             |      |
 *     M4 -> |          | <- M3
 *    (ENB)  |          |   (ENC)
 * (IN3,IN4) |          | (IN4,IN5)
 *             |      |
 *              ------
 *         [Մարսագնացի հետևը]
*/

#define DEBUG

#define ENA 9   // PWM control for Motor 6
#define IN1 13  // Motor 6 direction pin 1
#define IN2 12  // Motor 6 direction pin 2

#define ENB 8   // PWM control for Motors 4 & 5
#define IN3 11  // Motors 4 & 5 direction pin 1
#define IN4 10  // Motors 4 & 5 direction pin 2

#define ENC 3  // PWM control for Motor 1
#define IN5 7  // Motor 1 direction pin 1
#define IN6 6  // Motor 1 direction pin 2

#define END 2  // PWM control for Motors 2 & 3
#define IN7 5  // Motors 2 & 3 direction pin 1
#define IN8 4  // Motors 2 & 3 direction pin 2

#define MOTOR_SPEED 120  // Define motor speed

#define TRIG_PIN_FRONT 45  // Define the front ultrasonic sensor TRIG pin
#define ECHO_PIN_FRONT 44  // Define the front ultrasonic sensor ECHO pin
#define TRIG_PIN_LEFT 22   // Define the left ultrasonic sensor TRIG pin
#define ECHO_PIN_LEFT 23   // Define the left ultrasonic sensor ECHO pin
#define TRIG_PIN_RIGHT 42  // Define the right ultrasonic sensor TRIG pin
#define ECHO_PIN_RIGHT 43  // Define the right ultrasonic sensor ECHO pin

// Define maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 200

NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// PID Parameters
double Setpoint, Input, Output;
double Kp = 6, Ki = 0, Kd = 4;  // We need to tune these parameters for robot.

// Define PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

#ifdef DEBUG
unsigned int last_distanceFront = 0;
unsigned int last_distanceLeft = 0;
unsigned int last_distanceRight = 0;
double lastOutput = 0;
#endif

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(END, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
  analogWrite(ENC, MOTOR_SPEED);
  analogWrite(END, MOTOR_SPEED);

  Setpoint = 0;                                      // We want the error to be zero (i.e., no difference between left and right distances)
  myPID.SetMode(AUTOMATIC);                          // Turn the PID on
  myPID.SetOutputLimits(-MOTOR_SPEED, MOTOR_SPEED);  // Limit the PID output to the range needed
}


void loop() {

  // Handle serial communication
  if (Serial.available()) {
    String setting = Serial.readStringUntil('\n');
    handleSerialData(setting);
  }

  unsigned int distanceFront = sonarFront.convert_cm(sonarFront.ping_median(5));
  unsigned int distanceLeft = sonarLeft.convert_cm(sonarLeft.ping_median(5));
  unsigned int distanceRight = sonarRight.convert_cm(sonarRight.ping_median(5));

#ifdef DEBUG
  unsigned int diff;

  diff = max(distanceFront, last_distanceFront) - min(distanceFront, last_distanceFront);
  if (diff > 5) {
    Serial.print("distanceFront\t-\t");
    Serial.println(distanceFront);
    last_distanceFront = distanceFront;
  }

  diff = max(distanceLeft, last_distanceLeft) - min(distanceLeft, last_distanceLeft);
  if (diff > 5) {
    Serial.print("distanceLeft\t-\t");
    Serial.println(distanceLeft);
    last_distanceLeft = distanceLeft;
  }

  diff = max(distanceRight, last_distanceRight) - min(distanceRight, last_distanceRight);
  if (diff > 5) {
    Serial.print("distanceRight\t-\t");
    Serial.println(distanceRight);
    last_distanceRight = distanceRight;
  }
#endif

  // Wall following PID control
  if (distanceFront > 25) {
    // Calculate the difference between the two side distances
    Input = (double)(distanceLeft - distanceRight);
    myPID.Compute();
    moveForwardPID(Output);
    // moveForwardPID(0);

#ifdef DEBUG
    if (abs(Output - lastOutput) > 0.5) {
      Serial.print("PID Output\t-\t");
      Serial.println(Output);
      lastOutput = Output;
    }
#endif

    if (distanceRight > 20 && distanceLeft > 20) {
      // TODO
      delay(3000);  // Task 1
      tasks++;      // First task completed
    }

    if (distanceLeft > 20 && distanceFront > 50 && tasks == 1) {
      delay(3000);  // TODO. Task2
      tasks++;      // Second task completed
    }

    if (distanceRight > 20 && distanceFront > 50 && tasks == 2) {
      delay(3000);  // TODO. Task3
      tasks++;      // All tasks completed
    }
  } else if (distanceRight > 20) {  // Clear path to the right
    turnRight();
    delay(500);
  } else if (distanceLeft > 20) {  // Clear path to the left
    turnLeft();
    delay(500);
  } else {  // No clear path, stop (or turn around?)
    stopMotors();
  }
}

void handleSerialData(String setting) {
  char command = setting.charAt(0);
  double value = setting.substring(2).toDouble();

  switch (command) {
    case 'P':
      Kp = value;
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("New Kp: ");
      Serial.println(Kp);
      break;
    case 'I':
      Ki = value;
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("New Ki: ");
      Serial.println(Ki);
      break;
    case 'D':
      Kd = value;
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.print("New Kd: ");
      Serial.println(Kd);
      break;
    case 'F':
      moveForwardPID(0);
      break;
    case 'B':
      moveBackward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'S':
      stopMotors();
    default:
      stopMotors();
      break;
  }
}

void moveForwardPID(double output) {
  int leftSpeed = MOTOR_SPEED + output;
  int rightSpeed = MOTOR_SPEED - output;

  // Make sure we don't try to write a negative voltage to the motor
  leftSpeed = constrain(leftSpeed, 0, MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MOTOR_SPEED);

  analogWrite(ENA, leftSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, leftSpeed);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENC, rightSpeed);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  analogWrite(END, rightSpeed);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

void turnLeft() {
  analogWrite(ENA, 200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, 200);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENC, 200);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  analogWrite(END, 200);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

void turnRight() {
  analogWrite(ENA, 200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, 200);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENC, 200);
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  analogWrite(END, 200);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

void stopMotors() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  digitalWrite(IN8, LOW);
  digitalWrite(IN7, LOW);
}