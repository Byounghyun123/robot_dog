#include <ESP32Servo.h>

Servo frontRightHip, frontRightKnee;
Servo frontLeftHip, frontLeftKnee;
Servo rearRightHip, rearRightKnee;
Servo rearLeftHip, rearLeftKnee;

#define FR_HIP_PIN 16
#define FR_KNEE_PIN 17
#define FL_HIP_PIN 18
#define FL_KNEE_PIN 19
#define RR_HIP_PIN 21
#define RR_KNEE_PIN 22
#define RL_HIP_PIN 23
#define RL_KNEE_PIN 25

void setup() {
    Serial.begin(115200);

    // Attach servos to GPIO pins
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
    float leg_positions[4][3] = {
        {0.1, -0.05, -0.15}, 
        {0.1, 0.05, -0.15},  
        {-0.1, -0.05, -0.15},
        {-0.1, 0.05, -0.15}
    };

    float angles[12];
    ik.inverseKinematics(leg_positions, 0, 0, 0, 0, 0, 0, angles);

    // Convert radians to degrees (and limit between 0-180)
    int servo_angles[8];
    for (int i = 0; i < 8; i++) {
        servo_angles[i] = constrain((angles[i] * 180.0 / PI) + 90, 0, 180);
    }

    // Move servos
    frontRightHip.write(servo_angles[0]);
    frontRightKnee.write(servo_angles[1]);
    frontLeftHip.write(servo_angles[3]);
    frontLeftKnee.write(servo_angles[4]);
    rearRightHip.write(servo_angles[6]);
    rearRightKnee.write(servo_angles[7]);
    rearLeftHip.write(servo_angles[9]);
    rearLeftKnee.write(servo_angles[10]);

    delay(1000);
}
