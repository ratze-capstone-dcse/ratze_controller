#pragma once
#include <Arduino.h>
// #include <bno055-heading.h>
#include <bno055-new.h> // uncomment if using full imu data ya
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
#define CMD_RESET_IMU 'C'
#define CMD_KP 'P'
#define CMD_KI 'I'
#define CMD_KD 'D'
#define CMD_INFO 'Q'

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
#define WHEEL_BASE 0.145   // in meters (distance between left and right wheels) 14,5cm

#define TURN_90_COUNTS ((WHEEL_BASE) / (4 * WHEEL_RADIUS * 2) * COUNTS_PER_REV)

void sendSensorData()
{
  // send imu data
  Serial.print("IMU,");
  Serial.print(rad_roll, 3);
  Serial.print(",");
  Serial.print(rad_pitch, 3);
  Serial.print(",");
  Serial.print(rad_yaw, 3);
  Serial.print(",");
  Serial.print(gyro_x, 3);
  Serial.print(",");
  Serial.print(gyro_y, 3);
  Serial.print(",");
  Serial.print(gyro_z, 3);
  Serial.print(",");
  Serial.print(_linear_acc.x, 3);
  Serial.print(",");
  Serial.print(_linear_acc.y, 3);
  Serial.print(",");
  Serial.print(_linear_acc.z, 3);
  Serial.println();

  // send Tof data
  Serial.print("TOF,");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(tof_distances[i]);
    if (i < NUM_SENSORS - 1)
      Serial.print(",");
  }
  Serial.println();

  // send encoder data
  Serial.print("ENC,");
  Serial.print(countM1_);
  Serial.print(",");
  Serial.print(countM2_);
  Serial.print(",");
  Serial.print(countM3_);
  Serial.print(",");
  Serial.print(countM4_);
  Serial.println();
}
void setMotorSpeed(int speed)
{
  if (speed <= 0)
  {
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
  isMoving = true;
}

void processCmd()
{
  char cmd = cmd_buffer[0];
  int value = 0;
  int value2 = 0;

  if (cmd_index > 2 && cmd_buffer[1] == ':')
  {
    value = atoi(cmd_buffer + 2);
    // Check for second value after another colon
    char *colon_pos = strchr(cmd_buffer + 2, ':');
    if (colon_pos != nullptr)
    {
      value2 = atoi(colon_pos + 1);
    }
  }

  switch (cmd)
  {
  case CMD_MOVE_FORWARD:
    if (value == 0 && value2 == 0)
    {
      cmd_vel_.x = 1.0;
      cmd_vel_.w = 0.0;
    }
    else
    {
      cmd_vel_.x = value;
      cmd_vel_.w = value2;
    }
    isMoving = true;
    pid_right.reset();
    pid_left.reset();
    Serial.println("ACK:F");
    break;
  case CMD_MOVE_BACKWARD:
    if (value == 0 && value2 == 0)
    {
      cmd_vel_.x = -1.0;
      cmd_vel_.w = 0.0;
    }
    else
    {
      cmd_vel_.x = value;
      cmd_vel_.w = value2;
    }
    isMoving = true;
    pid_right.reset();
    pid_left.reset();
    Serial.println("ACK:B");
    break;
  case CMD_TURN_RIGHT:
    if (value == 0 && value2 == 0)
    {
      cmd_vel_.x = 0.0;
      cmd_vel_.w = -4.0;
    }
    else
    {
      cmd_vel_.x = value;
      cmd_vel_.w = value2;
    }

    isMoving = false; // after turn stop
    Serial.println("ACK:R");
    pid_right.reset();
    pid_left.reset();
    break;
  case CMD_TURN_LEFT:
    if (value == 0 && value2 == 0)
    {
      cmd_vel_.x = 0.0;
      cmd_vel_.w = 4.0;
    }
    else
    {
      cmd_vel_.x = value;
      cmd_vel_.w = value2;
    }
    isMoving = false; // after turn stop
    Serial.println("ACK:L");
    pid_right.reset();
    pid_left.reset();
    break;
  case CMD_STOP:
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    pid_right.reset();
    pid_left.reset();
    // moveStop();
    isMoving = false;
    Serial.println("ACK:S");
    break;
  case CMD_SET_SPEED:
    setMotorSpeed(value);
    Serial.println("ACK:V");
    break;
  case CMD_RESET_ENCODERS:
    resetEncoders();
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    pid_right.reset();
    pid_left.reset();
    Serial.println("ACK:E");
    break;
  case CMD_RESET_IMU:
    init_imu();
    update_imu();
    // heading_offset = heading;
    Serial.println("ACK:C");
    break;
  case CMD_KP:
    MOTOR_DRIVER_PID_KP = value;
    update_pid_gain();
    Serial.println("==========================");
    Serial.println("Set KP to " + String(value));
    Serial.println("==========================");
    break;
  case CMD_KI:
    MOTOR_DRIVER_PID_KI = value;
    update_pid_gain();
    Serial.println("==========================");
    Serial.println("Set KI to " + String(value));
    Serial.println("==========================");
    break;
  case CMD_KD:
    MOTOR_DRIVER_PID_KD = value;
    update_pid_gain();
    Serial.println("==========================");
    Serial.println("Set KD to " + String(value));
    Serial.println("==========================");
    break;
  case CMD_INFO:
    Serial.println("==========================");
    Serial.println("Current speed M1: " + String(velocity_.M1));
    Serial.println("Current speed M2: " + String(velocity_.M2));
    Serial.println("==========================");
    break;
  default:
    Serial.print("ERR:Unknown command: ");
    Serial.println(cmd_buffer);
    break;
  }
  int value = 0;
  int value2 = 0;
}

// Setup function to be called from main.cpp
void setupFirmware()
{
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 3000)
    ;

  Serial.println("# RATZE Micromouse Initializing...");

  // Initialize IMU
  init_imu();
  Serial.println("# IMU initialized");

  // Initialize motors
  setupMotors();
  Serial.println("# Motors initialized");
  resetEncoders();

  pid_right.set_output_limits(-255, 255);
  pid_left.set_output_limits(-255, 255);

  // Initialize ToF sensors
  if (setupToF())
  {
    Serial.println("# ToF sensors initialized");
  }
  else
  {
    Serial.println("# WARNING: Some ToF sensors failed to initialize");
  }

  Serial.println("# Initialization complete");
  Serial.println("# Format: IMU,heading,roll,pitch,yaw,sys,gyro,accel,mag");
  Serial.println("# Format: TOF,s0,s1,s2,s3,s4,s5,s6");
  Serial.println("# Format: ENC,m1,m2,m3,m4");
  Serial.println("READY");
}

// Loop function to be called from main.cpp
void loopFirmware()
{
  // print sensor data
  if (millis() - last_cmd_time > 100)
  { // send data every 100ms
    // printToFReadings();
    // printEncoders();
    sendSensorData();
    last_cmd_time = millis();
  }

  // Process any incoming commands
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (cmd_index > 0)
      {
        cmd_buffer[cmd_index] = '\0';
        processCmd();
        cmd_index = 0;
      }
    }
    else if (cmd_index < sizeof(cmd_buffer) - 1)
    {
      cmd_buffer[cmd_index++] = c;
    }
    last_cmd_time = millis();
  }

  // Check for command timeout (safety stop)
  if (isMoving && (millis() - last_cmd_time > COMMAND_TIMEOUT))
  {
    moveStop();
    isMoving = false;
    Serial.println("# Command timeout - stopped");
  }

  // Update sensors at appropriate intervals
  static unsigned long last_sensor_time = 0;
  if (millis() - last_sensor_time >= 50)
  { // 20Hz update rate
    // extract_heading();
    update_imu();
    updateToFReadings();
    last_sensor_time = millis();
  }

  static unsigned long last_motor_time = 0;
  if (millis() - last_motor_time >= 10)
  {
    motor_loop();
    last_motor_time = millis();
  }
}