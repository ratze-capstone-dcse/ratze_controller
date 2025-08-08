#pragma once
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define NUM_SENSORS 7
#define TCA9548A_ADDR 0x70  

Adafruit_VL53L0X tof_sensors[NUM_SENSORS];
uint16_t tof_distances[NUM_SENSORS] = {0};

// Select which channel on the TCA9548A to enable
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;  
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Initialize the ToF sensors
bool setupToF() {
  Wire.begin(SDA_PIN, SCL_PIN);
  
  bool all_initialized = true;

  // Initialize each sensor on its multiplexer channel
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);  // Enable only channel i

    if (!tof_sensors[i].begin()) {
      Serial.print("VL53L0X not found on channel ");
      Serial.println(i);
      all_initialized = false;
    } else {
    //   tof_sensors[i].setMeasurementTimingBudget(33000); // 33ms for ~30Hz readings
      Serial.print("Sensor on channel ");
      Serial.print(i);
      Serial.println(" initialized.");
    }
  }
  
  return all_initialized;
}

// Update all ToF sensor readings
void updateToFReadings() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    tcaSelect(i);  // Switch to channel i

    VL53L0X_RangingMeasurementData_t measure;
    tof_sensors[i].rangingTest(&measure, false); 

    if (measure.RangeStatus != 4) {
      tof_distances[i] = measure.RangeMilliMeter;
    } else {
      Serial.println("Out of range");
    }
  }
}

// Get a specific sensor reading
uint16_t getToFDistance(int sensor_index) {
  if (sensor_index >= 0 && sensor_index < NUM_SENSORS) {
    return tof_distances[sensor_index];
  }
  return 0;
}

// Print all ToF readings for debugging
void printToFReadings() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(tof_distances[i]);
    Serial.println(" mm");
  }
  Serial.println("----------------------");
}