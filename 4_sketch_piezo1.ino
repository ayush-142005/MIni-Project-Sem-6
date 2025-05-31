#define PIEZO_SENSOR 34   
#define THRESHOLD 4000   
#define DEBOUNCE_TIME 300 

int stepCount = 0;
unsigned long lastStepTime = 0;

void setup() {
    Serial.begin(115200);  
    pinMode(PIEZO_SENSOR, INPUT);
}

void loop() {
    int sensorValue = analogRead(PIEZO_SENSOR);  
    unsigned long currentTime = millis();        

    
    if (sensorValue > THRESHOLD) {
    int sensorValue = analogRead(PIEZO_SENSOR);  
    float voltage = (sensorValue / 4095.0) * 3.3; 

    Serial.print("ADC Value: ");
    Serial.print(sensorValue);
    Serial.print("  |  Voltage: ");
    Serial.print(voltage, 2); 
    Serial.println(" V");
    delay(100); 
        
        if (currentTime - lastStepTime > DEBOUNCE_TIME) {
          if (sensorValue > THRESHOLD ){
            stepCount++;  
            lastStepTime = currentTime; 
            Serial.print("Step Count: ");
            Serial.println(stepCount);
        }
        else {
          lastStepTime = currentTime; 
        }
      }
    }

    delay(5); 
}

