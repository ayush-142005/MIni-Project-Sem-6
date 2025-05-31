#include <Wire.h>
#include <MPU6050_tockn.h>  // Compatible with MPU6500

MPU6050 mpu(Wire);

const float FALL_THRESHOLD = 180; // Adjust based on testing

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);  // SDA = 21, SCL = 22

    mpu.begin();
    mpu.calcGyroOffsets(true);  // Calibrate gyro
    Serial.println("MPU6500 Initialized...");
}

void loop() {
    mpu.update();

    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    float gyroZ = mpu.getGyroZ();
    Serial.println( gyroX );
    Serial.println( gyroY );
    Serial.println( gyroZ );
    // Calculate total angular velocity
    float total_gyro = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

    Serial.print("Gyro: ");
    Serial.print(total_gyro);
    Serial.println(" deg/s");

    // Fall detection logic
    if (total_gyro > FALL_THRESHOLD) {
        Serial.println("⚠️ FALL DETECTED! ⚠️");
        // You can add code here to send alerts via WiFi, Bluetooth, or Buzzer
    }

    delay(1000); // Adjust delay based on real-time needs
}

