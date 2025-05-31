#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MPU6050_tockn.h>  

// ---------- Sensor Pins ----------
#define SENSOR_1 34    // Piezoelectric sensor 1

// ---------- Thresholds & Timeouts ----------
#define PIEZO_THRESHOLD 4000  
#define STEP_TIMEOUT 300       // Minimum delay between steps (ms)
#define FALL_THRESHOLD 180.0   // Gyroscope threshold for fall detection

// ---------- Objects ----------
MPU6050 mpu(Wire);
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1); // Use Serial1 for GPS communication

// ---------- Variables ----------
int stepCount = 0;
unsigned long lastStepTime = 0;
float total_distance_km = 0.0;
float start_lat = 0.0, start_lng = 0.0;
bool start_position_saved = false;

// ---------- Distance Calculation ----------
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371.0; // Earth radius in km
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c; // Distance in km
}

// ---------- Setup ----------
void setup() {
    Serial.begin(115200);

    pinMode(SENSOR_1, INPUT);

    GPS_Serial.begin(9600, SERIAL_8N1, 35, 32); // GPS TX=35, RX=32
    Wire.begin(21, 22);                         // I2C (SDA=21, SCL=22)

    mpu.begin();
    mpu.calcGyroOffsets(true); // Gyroscope calibration

    Serial.println("System Initialized...");
}

// ---------- Main Loop ----------
void loop() {
    readPiezoSensors();
    readGPSModule();
    detectFall();
    delay(500); // Reduced delay for more real-time feel
}

// ---------- Functions ----------

// 📌 Read Piezo Sensors and Count Steps
void readPiezoSensors() {
    int sensor1Value = analogRead(SENSOR_1);
    unsigned long currentTime = millis();

    Serial.print("[Piezo] Sensor Value: ");
    Serial.println(sensor1Value);

    if (sensor1Value > PIEZO_THRESHOLD && (currentTime - lastStepTime > STEP_TIMEOUT)) {
        stepCount++;
        lastStepTime = currentTime;
        Serial.print("[Piezo] Step Count: ");
        Serial.println(stepCount);

        float theoretical_distance_km = stepCount * 0.00078; // Approximate step distance
        Serial.print("[Piezo] Theoretical Distance: ");
        Serial.print(theoretical_distance_km, 6);
        Serial.println(" km");
    }
}

// 📌 Read GPS Data and Calculate Distance
void readGPSModule() {
    while (GPS_Serial.available() > 0) {
        char c = GPS_Serial.read();
        if (gps.encode(c)) {
            if (gps.location.isValid()) {
                float lat = gps.location.lat();
                float lng = gps.location.lng();

                Serial.print("[GPS] Current Location: ");
                Serial.print(lat, 6);
                Serial.print(", ");
                Serial.println(lng, 6);

                if (!start_position_saved) {
                    start_lat = lat;
                    start_lng = lng;
                    start_position_saved = true;
                    Serial.println("[GPS] Start Position Saved!");
                }

                float distance_km = calculateDistance(start_lat, start_lng, lat, lng);

                if (distance_km > 0.02) { // Reduced GPS noise more strictly
                    total_distance_km += distance_km;
                    start_lat = lat;
                    start_lng = lng;
                }

                Serial.print("[GPS] Total Distance Traveled: ");
                Serial.print(total_distance_km, 6);
                Serial.println(" km");

                // Simple calories calculation (approx 50 kcal/km)
                float calories_burned = total_distance_km * 50;
                Serial.print("[GPS] Calories Burned: ");
                Serial.print(calories_burned, 2);
                Serial.println(" kcal");

                delay(1000); // Reduced delay for quicker GPS updates
            }
        }
    }
}

// 📌 Fall Detection Using Gyroscope
void detectFall() {
    mpu.update();

    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    float gyroZ = mpu.getGyroZ();

    float total_gyro = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

    Serial.print("[MPU6050] Gyro Magnitude: ");
    Serial.println(total_gyro);

    if (total_gyro > FALL_THRESHOLD) {
        Serial.println("⚠️ FALL DETECTED! ⚠️");
        Serial.println("Please press ENTER within 10 seconds if you are OK...");

        unsigned long fallTime = millis();
        bool falseAlarm = false;

        while (millis() - fallTime < 10000) { // 10 seconds window
            if (Serial.available()) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') { // User pressed Enter
                    falseAlarm = true;
                    break;
                }
            }
        }

        if (falseAlarm) {
            Serial.println("✅ False Alarm detected. No action needed.");
        } else {
            Serial.println("🚨 No response! Sending REAL Fall Alert!");


            if (gps.location.isValid()) {
                float lat = gps.location.lat();
                float lng = gps.location.lng();

                Serial.print("[Fall] Location: ");
                Serial.print(lat, 6);
                Serial.print(", ");
                Serial.println(lng, 6);

            }
        }
    }
}

