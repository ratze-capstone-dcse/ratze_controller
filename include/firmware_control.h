#pragma once
#include <Arduino.h>
#include <bno055-heading.h>
// #include <bno055-new.h> // uncomment if using full imu data ya
#include <Wire.h>
#include <motor_driver.h>
#include <motor_control.h>
#include <tof_main.h>
#include "header.h"

// Serial communication parameters
#define SERIAL_BAUD 115200
#define COMMAND_TIMEOUT 1000 // ms

// Command protocol definitions
#define CMD_MOVE_FORWARD 'F'
#define CMD_MOVE_BACKWARD 'B'
#define CMD_TURN_LEFT 'L'
#define CMD_TURN_RIGHT 'R'
#define CMD_STOP 'S'
#define CMD_SET_SPEED 'V'
#define CMD_GET_SENSORS 'G'
#define CMD_RESET_ENCODERS 'E'
#define CMD_CALIBRATE_HEADING 'C'

// Tof Setup
#define TCA9548A_ADDR 0x70
#define NUM_SENSORS 7
// Adafruit_VL53L0X tof_sensors[NUM_SENSORS];
// uint16_t tof_distances[NUM_SENSORS];

// command processing variables
char cmd_buffer[64];
int cmd_index = 0;
unsigned long last_cmd_time = 0;
bool isMoving = false;
int currentSpeed = 0;
static long long targetCounts = 0;


// 90 degree turn calculation with encoders
#define COUNTS_PER_REV 100
#define WHEEL_RADIUS 0.035 // in meters (30mm)
#define WHEEL_BASE 0.145 // in meters (distance between left and right wheels) 14,5cm

#define TURN_90_COUNTS ((WHEEL_BASE) / (4 * WHEEL_RADIUS * 2) * COUNTS_PER_REV)


void sendSensorData() {
    // send imu data
    Serial.print("IMU, ");
    Serial.print(corrected_heading, 2);
    Serial.print(',');

    // We can add roll, pitch, yaw if using the other BNO mode
    Serial.print(0.0, 2); Serial.print(","); // Roll placeholder
    Serial.print(0.0, 2); Serial.print(","); // Pitch placeholder
    Serial.print(0.0, 2); Serial.print(","); // Yaw placeholder
    Serial.print(sys_calib); Serial.print(",");
    Serial.print(gyro_calib); Serial.print(",");
    Serial.print(accel_calib); Serial.print(",");
    Serial.print(mag_calib);
    Serial.println();

    // send Tof data
    Serial.print("TOF,");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(tof_distances[i]);
        if (i < NUM_SENSORS - 1) Serial.print(",");
    }
    Serial.println();

    // send encoder data
    Serial.print("ENC,");
    Serial.print(countM1_); Serial.print(",");
    Serial.print(countM2_); Serial.print(",");
    Serial.print(countM3_); Serial.print(",");
    Serial.print(countM4_); 
    Serial.println();
}
void setMotorSpeed(int speed){
    if (speed <=0){
        moveStop;
        isMoving = false;
        currentSpeed = 0;
        return;
    }
    
    // apply new speed while mantaining direction
    analogWrite(EN_M1, speed);
    analogWrite(EN_M2, speed);
    analogWrite(EN_M3, speed);
    analogWrite(EN_M4, speed);

    currentSpeed = speed;
    isMoving = true ;
}

// for discrete turns with encoder
void turn90degrees(bool clockwise, int speed) {
  if (clockwise) {
    setTurnRight();
  } else {
    setTurnLeft();
  }

  int initialCountM1 = countM1_;
  int initialCountM2 = countM2_;
  int initialCountM3 = countM3_;
  int initialCountM4 = countM4_;
  
  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);

  targetCounts += TURN_90_COUNTS;  
  Serial.print("Target Counts for 90-degree turn: ");
  Serial.println(targetCounts);

  // while (abs(countM1_) < targetCounts && abs(countM2_) < targetCounts &&
  //        abs(countM3_) < targetCounts && abs(countM4_) < targetCounts) {
  //   // wait until the turn is complete
  //   delay(1);
  // }
  // if (clockwise) {
  //   while (abs(countM2_) < targetCounts && abs(countM4_) < targetCounts) {
  //     delay(1);
  //   }
  // } else {
  //   while (abs(countM1_) < targetCounts && abs(countM3_) < targetCounts) {
  //     delay(1);
  //   }
  // }

  while (true) {
    int deltaM1 = abs(countM1_ - initialCountM1);
    int deltaM2 = abs(countM2_ - initialCountM2);
    int deltaM3 = abs(countM3_ - initialCountM3);
    int deltaM4 = abs(countM4_ - initialCountM4);

    if (deltaM1 >= TURN_90_COUNTS && deltaM2 >= TURN_90_COUNTS &&
        deltaM3 >= TURN_90_COUNTS && deltaM4 >= TURN_90_COUNTS) {
      Serial.print("Delta M1: "); Serial.print(deltaM1);
      Serial.print(" Delta M2: "); Serial.print(deltaM2);
      Serial.print(" Delta M3: "); Serial.print(deltaM3);
      Serial.print(" Delta M4: "); Serial.println(deltaM4);
      break;
    }
    delay(1);
  }

  
  
  moveStop();
}

// for more precise turns with imu, unfortunately tends to overshoot
void turn90degrees_imu(bool clockwise, int speed) {
  float initialHeading = corrected_heading;
  float targetHeading = initialHeading + (clockwise ? 90.0 : -90.0);

  if (targetHeading >= 360.0) targetHeading -= 360.0;
  if (targetHeading < 0.0) targetHeading += 360.0;

  if (clockwise) {
    setTurnRight();
  } else {
    setTurnLeft();
  }

  // start turning

  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);

  while (true) {
    extract_heading();
    float currentHeading = corrected_heading;

    if (clockwise) {
      if ((initialHeading < targetHeading && currentHeading >= targetHeading) ||
          (initialHeading > targetHeading && (currentHeading >= targetHeading || currentHeading < initialHeading))) {
        break;
      }
    } else {
      if ((initialHeading > targetHeading && currentHeading <= targetHeading) ||
          (initialHeading < targetHeading && (currentHeading <= targetHeading || currentHeading > initialHeading))) {
        break;
      }
    }
    delay(10); // small delay to avoid busy-waiting
  }
  moveStop();
}

void processCmd(){
    char cmd = cmd_buffer[0];
    char* arg = cmd_buffer + 1;
    int value = atoi(arg);

    switch (cmd) {
        case CMD_MOVE_FORWARD:
            // moveForward(value > 0 ? value : 150);
            cmd_vel_.x = 1.0;
            cmd_vel_.w = 0.0;
            isMoving = true;
            Serial.println("ACK:F");
            break;
        case CMD_MOVE_BACKWARD:
            moveBackward(value > 0 ? value : 150);
            isMoving = true;
            Serial.println("ACK:B");
            break;
        case CMD_TURN_RIGHT:
            // turnRight(value > 0 ? value : 150);
            // isMoving = true;
            // turn90degrees_imu(true, value > 0 ? value : 200);
            // turn90degrees(true, value > 0 ? value : 225);
            cmd_vel_.x = 0.0;
            cmd_vel_.w = -1.0;
            

            isMoving = false; // after turn stop
            Serial.println("ACK:R");
            break;
        case CMD_TURN_LEFT:
            // turnLeft(value > 0 ? value : 150);
            // isMoving = true;
            // turn90degrees_imu(false, value > 0 ? value : 200);
            // turn90degrees(false, value > 0 ? value : 225);
            cmd_vel_.x = 0.0;
            cmd_vel_.w = 1.0;
            isMoving = false; // after turn stop
            Serial.println("ACK:L");
            break;
        case CMD_STOP:
            // cmd_vel_.x = 0.0;
            // cmd_vel_.w = 0.0;
            moveStop();
            isMoving = false;
            Serial.println("ACK:S");
            break;
        case CMD_SET_SPEED:
            setMotorSpeed(value);
            Serial.println("ACK:V");
            break;
        case CMD_RESET_ENCODERS:
            resetEncoders();
            Serial.println("ACK:E");
            break;
        case CMD_CALIBRATE_HEADING:
            heading_offset = heading;
            Serial.println("ACK:C");
            break;  
        default:
            Serial.print("ERR:Unknown command: ");
            Serial.println(cmd_buffer);
            break;
    }
}

// Setup function to be called from main.cpp
void setupFirmware() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000);
  
  Serial.println("# RATZE Micromouse Initializing...");
  
  // Initialize IMU
  init_imu();
  Serial.println("# IMU initialized");
  
  // Initialize motors
  setupMotors();
  Serial.println("# Motors initialized");
  resetEncoders();
  
  // Initialize ToF sensors
  if (setupToF()) {
    Serial.println("# ToF sensors initialized");
  } else {
    Serial.println("# WARNING: Some ToF sensors failed to initialize");
  }
  
  Serial.println("# Initialization complete");
  Serial.println("# Format: IMU,heading,roll,pitch,yaw,sys,gyro,accel,mag");
  Serial.println("# Format: TOF,s0,s1,s2,s3,s4,s5,s6");
  Serial.println("# Format: ENC,m1,m2,m3,m4");
  Serial.println("READY");
}

// Loop function to be called from main.cpp
void loopFirmware() {
  // print sensor data
  if (millis() - last_cmd_time > 500) { // send data every 500ms
    // printToFReadings();
    // printEncoders();
    sendSensorData();
    last_cmd_time = millis();
  }

  // Process any incoming commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_index > 0) {
        cmd_buffer[cmd_index] = '\0';
        processCmd();
        cmd_index = 0;
      }
    } else if (cmd_index < sizeof(cmd_buffer) - 1) {
      cmd_buffer[cmd_index++] = c;
    }
    last_cmd_time = millis();
  }

  // Check for command timeout (safety stop)
  if (isMoving && (millis() - last_cmd_time > COMMAND_TIMEOUT)) {
    moveStop();
    isMoving = false;
    Serial.println("# Command timeout - stopped");
  }

  // Update sensors at appropriate intervals
  static unsigned long last_sensor_time = 0;
  if (millis() - last_sensor_time >= 50) { // 20Hz update rate
    extract_heading();
    updateToFReadings();
    last_sensor_time = millis();
  }

  static unsigned long last_control_time = 0;
  if (millis() - last_control_time >= 10) { // 100Hz update rate
    motor_loop();
    last_control_time = millis();
  }

}