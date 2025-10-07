#pragma once
#include "pid.hpp"

// Motor pin definitions
#define M1_IN1 26 //26
#define M1_IN2 25 //25
#define M2_IN1 33 //33
#define M2_IN2 32 //32
#define M3_IN1 13 //13
#define M3_IN2 12 //12
#define M4_IN1 14 //14
#define M4_IN2 27 //27

#define EN_M1 23
#define EN_M2 19
#define EN_M3 5 // 18
#define EN_M4 18 // 5

#define ENC_M1_A 4 
#define ENC_M1_B 15
#define ENC_M2_A 34
#define ENC_M2_B 35
#define ENC_M3_A 36
#define ENC_M3_B 39
#define ENC_M4_A 17
#define ENC_M4_B 16

#define MAX_SPEED 255
#define STEP_DELAY 5

#define TICKS_PER_REV 100
#define WHEEL_RADIUS 0.035 // in meters (30mm)
#define WHEEL_BASE 0.145   // in meters (distance between left and right wheels) 14,5cm
#define TURN_90_COUNTS ((WHEEL_BASE) / (4 * WHEEL_RADIUS * 2) * TICKS_PER_REV)

PID pid_right(0.0, 0.0, 0.0);
PID pid_left(0.0, 0.0, 0.0);

int MOTOR_DRIVER_PID_KP = 200.0;
int  MOTOR_DRIVER_PID_KI = 70.0;
int MOTOR_DRIVER_PID_KD = 30.0;

#define MOTOR_MAX_VELOCITY 1.0  // Maximum velocity in m/s

struct {
    float x;
    float w;
} cmd_vel_;

// Encoder counts
volatile long countM1_ = 0;
volatile long countM2_ = 0;
volatile long countM3_ = 0;
volatile long countM4_ = 0;

struct {
    unsigned long M1;
    unsigned long M2;
    unsigned long M3;
    unsigned long M4;
} last_data_reading_time_;

struct {
    int32_t M1;
    int32_t M2;
    int32_t M3;
    int32_t M4;
} last_encoder_reading_;

struct {
    float M1;
    float M2;
    float M3;
    float M4;
} rpm_;

struct {
    float M1;
    float M2;
    float M3;
    float M4;
} angular_velocity_;

struct {
    float M1;
    float M2;
    float M3;
    float M4;
} velocity_;

struct {
    float M1;
    float M2;
    float M3;
    float M4;
} distance_;

void resetEncoders();