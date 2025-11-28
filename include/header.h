#pragma once
#include "pid.hpp"
#include "timer_api.hpp"

// Motor pin definitions
#define M1_IN1 26 // 26
#define M1_IN2 25 // 25
#define M2_IN1 33 // 33
#define M2_IN2 32 // 32
#define M3_IN1 13 // 13
#define M3_IN2 12 // 12
#define M4_IN1 14 // 14
#define M4_IN2 27 // 27

#define EN_M1 23
#define EN_M2 19
#define EN_M3 5  // 18
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

#define MOTOR_PWM_MAX 255.0f
#define MOTOR_PWM_DEADBAND 200.0f

#define TICKS_PER_REV 100
#define WHEEL_RADIUS 0.035 // in meters (30mm)
#define WHEEL_BASE 0.15   // in meters (distance between front and rear wheels)
#define WHEEL_TRACK 0.26
#define TURN_90_COUNTS ((WHEEL_BASE) / (4 * WHEEL_RADIUS * 2) * TICKS_PER_REV)

PID pid_right(80.0, 10.0, 30.0);
PID pid_left(80.0, 10.0, 30.0); 

int MOTOR_DRIVER_PID_KP = 200.0;
int MOTOR_DRIVER_PID_KI = 70.0;
int MOTOR_DRIVER_PID_KD = 30.0;

float MOTOR_PWM_SCALE_RIGHT = 1.0f;
float MOTOR_PWM_SCALE_LEFT = 1.0f;
float MOTOR_PWM_BIAS_RIGHT = 0.0f;
float MOTOR_PWM_BIAS_LEFT = 0.0f;

inline unsigned long hz_to_ms(uint8_t hz) { return 1000 / hz; }

TimerAPI motor_update_timer_(hz_to_ms(50));

#define MOTOR_MAX_VELOCITY 1.0 // Maximum velocity in m/s

// Wall following parameters
#define LOOP_MS 20                // Main loop period in ms (50Hz)
#define ALPHA 0.3                 // Low-pass filter coefficient (0-1, higher = more responsive)
#define FRONT_THRESHOLD 350       // Front wall detection threshold in mm (increased for safety)
#define SIDE_WALL_THRESHOLD 400   // Side wall detection threshold in mm (no wall if > this)
#define BASE_SPEED 130            // Base motor speed (0-255) - reduced for better control
#define TURN_SPEED 200            // Speed when turning to find wall
#define MAX_CORR 50               // Maximum correction value
#define Kp 0.2                    // Proportional gain for wall following
const int TURN_DELAY = 400;       // 90-degree turn duration in ms
const int RIGHT_TURN_DURATION = 300;  // Duration for right turn when no wall (ms)
const int LEFT_TURN_DURATION = 300;   // Duration for left turn when no wall (ms)
const int FORWARD_AFTER_TURN_DURATION = 400; // Duration to move forward after turn (ms)

const int RIGHT_TURN_DELAY = 1100;
const int LEFT_TURN_DELAY = 900;

// Cell-based navigation parameters
#define CELL_SIZE_MM 300.0              // Standard micromouse cell size (300mm = 30cm)
#define CELL_SIZE_M 0.30                // Cell size in meters
#define WHEEL_CIRCUMFERENCE (2.0 * PI * WHEEL_RADIUS)  // Meters per revolution
#define TICKS_PER_METER (TICKS_PER_REV / WHEEL_CIRCUMFERENCE)
#define TICKS_PER_CELL (TICKS_PER_METER * CELL_SIZE_M)  // Encoder ticks for one cell

// Intersection detection thresholds
#define INTERSECTION_DETECT_THRESHOLD 250  // Distance at which to detect intersection (mm)
#define MIN_CELL_DISTANCE 0.8              // Minimum distance before detecting next intersection (80% of cell)

struct
{
    float x;
    float w;
} cmd_vel_;

// Encoder counts
volatile long countM1_ = 0;
volatile long countM2_ = 0;
volatile long countM3_ = 0;
volatile long countM4_ = 0;

struct
{
    unsigned long M1;
    unsigned long M2;
    unsigned long M3;
    unsigned long M4;
} last_data_reading_time_;

struct
{
    int32_t M1;
    int32_t M2;
    int32_t M3;
    int32_t M4;
} last_encoder_reading_;

struct
{
    float M1;
    float M2;
    float M3;
    float M4;
} rpm_;

struct
{
    float M1;
    float M2;
    float M3;
    float M4;
} angular_velocity_;

struct
{
    float M1;
    float M2;
    float M3;
    float M4;
} velocity_;

struct
{
    float M1;
    float M2;
    float M3;
    float M4;
} distance_;

void resetEncoders();