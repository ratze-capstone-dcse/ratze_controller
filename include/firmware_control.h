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
#define CMD_WALL_FOLLOW 'W'

// Tof Setup
#define TCA9548A_ADDR 0x70
#define NUM_SENSORS 7
// Adafruit_VL53L0X tof_sensors[NUM_SENSORS];
// uint16_t tof_distances[NUM_SENSORS];



// Wall following state variables
bool wallFollowEnabled = false;
unsigned long lastWallFollowMillis = 0;
float filtL = 0.0;                // Filtered left sensor reading
float filtR = 0.0;                // Filtered right sensor reading

// Turn state management
enum TurnState {
  TURN_NONE,
  TURN_RIGHT,
  TURN_LEFT,
  TURN_FRONT
};

TurnState currentTurnState = TURN_NONE;
unsigned long turnStartTime = 0;
int turnDuration = 0;

// command processing variables
char cmd_buffer[64];
int cmd_index = 0;
unsigned long last_cmd_time = 0;
bool isMoving = false;
int currentSpeed = 0;
static long long targetCounts = 0;

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

void wallFollowingLoop()
{
  unsigned long now = millis();
  
  // Check if we're currently executing a turn
  if (currentTurnState != TURN_NONE)
  {
    unsigned long elapsed = now - turnStartTime;
    
    if (elapsed < turnDuration)
    {
      // Continue turning
      switch (currentTurnState)
      {
        case TURN_RIGHT:
          sendPWM(-TURN_SPEED, TURN_SPEED);
          break;
        case TURN_LEFT:
          sendPWM(TURN_SPEED, -TURN_SPEED);
          break;
        case TURN_FRONT:
          sendPWM(225, -225); // Front 90-degree turn
          break;
        default:
          break;
      }
      return; // Stay in turn mode
    }
    else
    {
      // Turn complete
      Serial.print("Turn complete: ");
      Serial.println(currentTurnState == TURN_RIGHT ? "RIGHT" : 
                     currentTurnState == TURN_LEFT ? "LEFT" : "FRONT");
      sendPWM(0, 0);
      delay(50); // Brief pause after turn
      currentTurnState = TURN_NONE;
      
      // Reset filters after turn
      filtL = 0;
      filtR = 0;
      return;
    }
  }
  
  // Normal control loop timing
  if (now - lastWallFollowMillis < LOOP_MS)
    return;
  lastWallFollowMillis = now;

  // Read sensor values (assuming sensor indices: 0=left, 2=right, 5=front)
  // Adjust indices based on your actual sensor placement
  int rawL = tof_distances[0]; // Left sensor
  int rawR = tof_distances[2]; // Right sensor
  int rawF = tof_distances[5]; // Front sensor

  // Apply low-pass filter to reduce noise
  filtL = ALPHA * rawL + (1.0 - ALPHA) * filtL;
  filtR = ALPHA * rawR + (1.0 - ALPHA) * filtR;

  // Debug output
  Serial.print("WF: L=");
  Serial.print(filtL);
  Serial.print(" R=");
  Serial.print(filtR);
  Serial.print(" F=");
  Serial.println(rawF);

  // --- Front wall check (highest priority) ---
  if (rawF < FRONT_THRESHOLD && rawF > 0)
  {
    Serial.println("Front wall detected - initiating left turn");
    currentTurnState = TURN_FRONT;
    turnStartTime = now;
    turnDuration = TURN_DELAY;
    sendPWM(0, 0); // Stop first
    delay(100);
    turnStartTime = millis(); // Reset after delay
    return;
  }

  // --- Wall detection logic ---
  // Check if walls are detected on left and right
  bool hasRightWall = (filtR > 0 && filtR < SIDE_WALL_THRESHOLD);
  bool hasLeftWall = (filtL > 0 && filtL < SIDE_WALL_THRESHOLD);

  // Priority: No right wall -> turn right
  if (!hasRightWall)
  {
    Serial.println("No right wall - initiating right turn");
    currentTurnState = TURN_RIGHT;
    turnStartTime = now;
    turnDuration = RIGHT_TURN_DURATION;
    return;
  }
  // Second priority: No left wall -> turn left
  else if (!hasLeftWall)
  {
    Serial.println("No left wall - initiating left turn");
    currentTurnState = TURN_LEFT;
    turnStartTime = now;
    turnDuration = LEFT_TURN_DURATION;
    return;
  }
  // Both walls present: Wall following mode
  else
  {
    // --- Normal wall following ---
    // Error is positive when too close to right wall (need to turn left)
    float error = filtR - filtL;
    float corr = Kp * error;
    corr = constrain(corr, -MAX_CORR, MAX_CORR);

    // Apply correction: add to left, subtract from right to turn away from right wall
    int leftPWM = BASE_SPEED + corr;
    int rightPWM = BASE_SPEED - corr;
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    sendPWM(leftPWM, rightPWM);
  }
}

void processCmd()
{
  char cmd = cmd_buffer[0];
  float value = 0;
  float value2 = 0;

  if (cmd_index > 2 && cmd_buffer[1] == ':')
  {
    value = atof(cmd_buffer + 2);
    // Check for second value after another colon
    char *colon_pos = strchr(cmd_buffer + 2, ':');
    if (colon_pos != nullptr)
    {
      value2 = atof(colon_pos + 1);
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
    Serial.println("==========================");
    Serial.println("CMD_X: " + String(value));
    Serial.println("CMD_W: " + String(value2));
    Serial.println("==========================");
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
  case CMD_WALL_FOLLOW:
    if (value == 0)
    {
      wallFollowEnabled = false;
      currentTurnState = TURN_NONE;
      moveStop();
      Serial.println("Wall following disabled");
    }
    else
    {
      wallFollowEnabled = true;
      filtL = 0;
      filtR = 0;
      currentTurnState = TURN_NONE;
      lastWallFollowMillis = millis();
      Serial.println("Wall following enabled");
    }
    Serial.println("ACK:W");
    break;
  default:
    Serial.print("ERR:Unknown command: ");
    Serial.println(cmd_buffer);
    break;
  }
  value = 0;
  value2 = 0;
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

  // Wall following control loop
  if (wallFollowEnabled)
  {
    wallFollowingLoop();
  }
}