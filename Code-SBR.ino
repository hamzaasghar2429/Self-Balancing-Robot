#include <PID_v1.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// L298N Motor Pins
const int IN1 = 6, IN2 = 7, IN3 = 8, IN4 = 9, ENA = 5, ENB = 10;

// PID Variables
double setpoint = 0;   // The "Zero" balance angle (Adjust after calibration)
double input, output;
double Kp = 15 , Ki = 100, Kd = 1.2 ; // Start with these, then tune
// ( kp = how fast the robot tilt form its highest point )
// ( ki Integral  long term error)
// ( kd how fast the rpbot is falling )
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
MPU6050 mpu6050(Wire);

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  Wire.begin();
  Serial.begin(115200);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // Calibrate on startup - keep robot still!

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-155, 100); // PWM range
  myPID.SetSampleTime(10);           // 100Hz refresh rate
}

void loop() {
  mpu6050.update();
  input = mpu6050.getAngleY(); // Reading the tilt angle

  if (abs(input) > 45) { // Safety: stop motors if robot falls
    stopMotors();
  } else {
    myPID.Compute();
    controlMotors(output);
  }
}

void controlMotors(double speed) {
  int pwm = abs(speed);
  if (speed > 0) { // Forward
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else { // Backward
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}