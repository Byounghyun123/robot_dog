#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>

// Servo objects for each leg (hip and knee servos)
Servo frontRightHip, frontRightKnee;
Servo frontLeftHip, frontLeftKnee;
Servo rearRightHip, rearRightKnee;
Servo rearLeftHip, rearLeftKnee;

// Define the GPIO pins for the servos.
#define FR_HIP_PIN 33   // first green wire from front
#define FR_KNEE_PIN 32  // black wire
#define FL_HIP_PIN 27   // orange wire
#define FL_KNEE_PIN 14  // second green wire
#define RR_HIP_PIN 25   // first yellow wire
#define RR_KNEE_PIN 26  // brown wire
#define RL_HIP_PIN 12   // second yellow wire
#define RL_KNEE_PIN 13  // third yellow wire

void setup() {
  Serial.begin(115200);

  // Attach servos to their respective GPIO pins.
  frontRightHip.attach(FR_HIP_PIN);
  frontRightKnee.attach(FR_KNEE_PIN);
  frontLeftHip.attach(FL_HIP_PIN);
  frontLeftKnee.attach(FL_KNEE_PIN);
  rearRightHip.attach(RR_HIP_PIN);
  rearRightKnee.attach(RR_KNEE_PIN);
  rearLeftHip.attach(RL_HIP_PIN);
  rearLeftKnee.attach(RL_KNEE_PIN);
}

void loop() {
  // ---------------- Knee Movement Only ----------------
  // Knees Move
  frontRightKnee.write(85);
  frontLeftKnee.write(100);
  rearRightKnee.write(85);
  rearLeftKnee.write(100);

  delay(200);

  // Knees Stop Move
  frontRightKnee.write(90);
  frontLeftKnee.write(90);
  rearRightKnee.write(90);
  rearLeftKnee.write(90);

  delay(200);

  // Knees Move other direction
  frontRightKnee.write(100);
  frontLeftKnee.write(85);
  rearRightKnee.write(100);
  rearLeftKnee.write(85);

  delay(200);

  // // ---------------- Hip Movement Only ----------------
  // // Hips Move
  // frontRightHip.write(85);
  // frontLeftHip.write(100);
  // rearRightHip.write(85);
  // rearLeftHip.write(100);

  // delay(200);

  // // Hips Stop Move
  // frontRightHip.write(90);
  // frontLeftHip.write(90);
  // rearRightHip.write(90);
  // rearLeftHip.write(90);

  // delay(200);

  // // Hips Move other direction
  // frontRightHip.write(100);
  // frontLeftHip.write(85);
  // rearRightHip.write(100);
  // rearLeftHip.write(85);

  // delay(200);

  // // ---------------- All Movement ----------------
  // // Knees Move
  // frontRightKnee.write(85);
  // frontLeftKnee.write(100);
  // rearRightKnee.write(85);
  // rearLeftKnee.write(100);
  // frontRightHip.write(85);
  // frontLeftHip.write(100);
  // rearRightHip.write(85);
  // rearLeftHip.write(100);

  // delay(200);

  // // Knees Stop Move
  // frontRightKnee.write(90);
  // frontLeftKnee.write(90);
  // rearRightKnee.write(90);
  // rearLeftKnee.write(90);
  // frontRightHip.write(90);
  // frontLeftHip.write(90);
  // rearRightHip.write(90);
  // rearLeftHip.write(90);

  // delay(200);

  // // Knees Move other direction
  // frontRightKnee.write(100);
  // frontLeftKnee.write(85);
  // rearRightKnee.write(100);
  // rearLeftKnee.write(85);
  // frontRightHip.write(100);
  // frontLeftHip.write(85);
  // rearRightHip.write(100);
  // rearLeftHip.write(85);

  // delay(200);
}
