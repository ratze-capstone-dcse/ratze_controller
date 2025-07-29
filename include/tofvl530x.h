#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define NUM_SENSORS 7

#define TCA9548A_ADDR 0x70  

Adafruit_VL53L0X sensors[NUM_SENSORS];

// Select which channel on the TCA9548A to enable
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;  
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize each sensor on its multiplexer channel
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);  // Enable only channel i

    if (!sensors[i].begin()) {
      Serial.print("VL53L0X not found on channel ");
      Serial.println(i);
      while (1);  // Stop if any sensor is missing
    } else {
      Serial.print("Sensor on channel ");
      Serial.print(i);
      Serial.println(" initialized.");
    }
  }
}

void loop() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);  // Switch to channel i

    VL53L0X_RangingMeasurementData_t measure;
    sensors[i].rangingTest(&measure, false); 

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    if (measure.RangeStatus != 4) {
      Serial.print(measure.RangeMilliMeter);
      Serial.println(" mm");
    } else {
      Serial.println("Out of range");
    }

    delay(50);
  }

  Serial.println("----------------------");
  delay(200);
}
