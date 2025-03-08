#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>

#define PI 3.14159265358979323846

// Structure to hold body and leg dimensions and perform inverse kinematics calculations.
struct InverseKinematics {
  float bodyLength;
  float bodyWidth;
  float l1, l2, l3, l4;

  InverseKinematics(float bLength, float bWidth, float leg1, float leg2, float leg3, float leg4) {
    bodyLength = bLength;
    bodyWidth = bWidth;
    l1 = leg1;
    l2 = leg2;
    l3 = leg3;
    l4 = leg4;
  }

  // Compute joint angles for each leg based on leg positions and additional offsets.
  // The results are stored in the angles array (3 angles per leg, total 12 angles).
  void inverseKinematics(float leg_positions[4][3], float dx, float dy, float dz,
                         float roll, float pitch, float yaw, float angles[12]) {
    int cnt = 0;
    for (int i = 0; i < 4; i++) {
      float x = leg_positions[i][0];
      float y = leg_positions[i][1];
      float z = leg_positions[i][2];

      float F = sqrt(x * x + y * y - l2 * l2);
      float G = F - l1;
      float H = sqrt(G * G + z * z);

      float theta1 = -atan2(y, x) - atan2(F, l2 * pow(-1, i));
      float D = (H * H - l3 * l3 - l4 * l4) / (2 * l3 * l4);
      float theta4 = acos(-D) - PI;
      float theta3 = atan2(z, G) - atan2(l4 * sin(theta4), l3 + l4 * cos(theta4));

      if (cnt == 0 || cnt == 2) {
        theta1 = -theta1;
      }

      angles[cnt * 3]     = theta1;
      angles[cnt * 3 + 1] = theta3;
      angles[cnt * 3 + 2] = theta4;
      cnt++;
    }
  }
};

// Create a global inverse kinematics instance with example dimensions.
InverseKinematics ik(0.2, 0.15, 0.05, 0.08, 0.1, 0.1);

// Servo objects for each leg (hip and knee servos)
Servo frontRightHip, frontRightKnee;
Servo frontLeftHip, frontLeftKnee;
Servo rearRightHip, rearRightKnee;
Servo rearLeftHip, rearLeftKnee;

// Define the GPIO pins for the servos.
#define FR_HIP_PIN 14   // first green wire from front
#define FR_KNEE_PIN 13  // black wire
#define FL_HIP_PIN 26   // orange wire
#define FL_KNEE_PIN 25  // second green wire
#define RR_HIP_PIN 27   // first yellow wire
#define RR_KNEE_PIN 12  // brown wire
#define RL_HIP_PIN 33   // second yellow wire
#define RL_KNEE_PIN 32  // third yellow wire

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
  // Example leg positions (X, Y, Z) for the 4 legs.
  float leg_positions[4][3] = {
    { 0.1, -0.05, -0.15 }, // Front Right
    { 0.1,  0.05, -0.15 }, // Front Left
    { -0.1, -0.05, -0.15 }, // Rear Right
    { -0.1,  0.05, -0.15 }  // Rear Left
  };

  float angles[12];  // Array to hold the computed IK joint angles.
  ik.inverseKinematics(leg_positions, 0, 0, 0, 0, 0, 0, angles);

  // Print the calculated joint angles (in radians) to the Serial Monitor.
  Serial.println("Joint Angles (Radians):");
  for (int i = 0; i < 12; i++) {
    Serial.print("Angle ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(angles[i], 4);
  }

  // Convert selected IK angles from radians to degrees for servo control.
  // We assume the hip and knee servos for each leg use the first two angles of each leg's set:
  // Front Right: indices 0 and 1; Front Left: indices 3 and 4;
  // Rear Right: indices 6 and 7; Rear Left: indices 9 and 10.
  int servoAngles[12];
  for (int i = 0; i < 12; i++) {
    servoAngles[i] = constrain((angles[i] * 180.0 / PI) + 90, 0, 180);
  }

  // Update servos with the calculated positions.
  frontRightHip.write(servoAngles[0]);
  frontRightKnee.write(servoAngles[1]);

  frontLeftHip.write(servoAngles[3]);
  frontLeftKnee.write(servoAngles[4]);

  rearRightHip.write(servoAngles[6]);
  rearRightKnee.write(servoAngles[7]);

  rearLeftHip.write(servoAngles[9]);
  rearLeftKnee.write(servoAngles[10]);

  delay(1000);
}

