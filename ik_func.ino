#include <Arduino.h>
#include <math.h>

#define PI 3.14159265358979323846

// Structure to hold body and leg dimensions
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

            angles[cnt * 3] = theta1;
            angles[cnt * 3 + 1] = theta3;
            angles[cnt * 3 + 2] = theta4;
            cnt++;
        }
    }
};

// Initialize inverse kinematics with body and leg dimensions
InverseKinematics ik(0.2, 0.15, 0.05, 0.08, 0.1, 0.1); // Example values

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Example leg positions (X, Y, Z) for 4 legs
    float leg_positions[4][3] = {
        {0.1, -0.05, -0.15}, // Front Right
        {0.1, 0.05, -0.15},  // Front Left
        {-0.1, -0.05, -0.15}, // Rear Right
        {-0.1, 0.05, -0.15}  // Rear Left
    };

    float angles[12];
    ik.inverseKinematics(leg_positions, 0, 0, 0, 0, 0, 0, angles);

    // Print calculated joint angles
    Serial.println("Joint Angles (Radians):");
    for (int i = 0; i < 12; i++) {
        Serial.print("Angle ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(angles[i], 4);
    }

    delay(1000);
}
