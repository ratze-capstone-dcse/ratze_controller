#pragma once
#include <Arduino.h>
#include <Wire.h>
// #include <AP_Math.h>
#include <BNO055_support.h> 
#include <I2Cdev.h>
// Representation Unit Settings
#define ACCEL_TO_MS2        100
#define ACCEL_TO_MG         1
#define MAG_TO_MIKROTESLA   16
#define GYRO_TO_DPS         16
#define GYRO_TO_RPS         900
#define EULER_TO_DEG        16
#define EULER_TO_RAD        900
#define QUAT_UNIT_LESS      16384
#define GRAVITY_TO_MS2      100
#define GRAVITY_TO_MG       1
#define TEMP_TO_CELC        1
#define TEMP_TO_FAHR        2

struct Vector3f {
    float x;
    float y;
    float z;

    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

Vector3f _linear_acc;  // Vector to hold the linear accelerometer data
Vector3f _accel;  // Vector to hold the accelerometer data

// Global variables for Euler angles and Gyroscope data
float roll = 0.0, pitch = 0.0, yaw = 0.0, heading = 0.0;
float rad_roll = 0.0, rad_pitch = 0.0, rad_yaw = 0.0;
float last_roll = 0.0, last_pitch = 0.0, last_yaw = 0.0;
float delta_roll = 0.0, delta_pitch = 0.0, delta_yaw = 0.0;
float gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0; 
// float heading_offset = 0.0;
unsigned char gyro_calib, accel_calib;


struct bno055_t bno_data;  // Structure to hold the BNO055 device information
struct bno055_euler bno_euler;
struct bno055_gyro bno_gyro;  // Structure to hold the Gyroscope data
struct bno055_accel bno_accel;  // Structure to hold the Accelerometer data
struct bno055_linear_accel bno_linear;  // Structure to hold the Linear Accelerometer data
unsigned char mag_calib_status = 0;  // Variable to hold the calibration status of the Magnetometer
unsigned char sys_calib_status = 0;  // Variable to hold the calibration status of the System (BNO055's MCU)
unsigned long last_time = 0;  // Variable to hold the last time stamp for calibration checks


void check_imu_calibration() {
    while (mag_calib_status != 3 || sys_calib_status != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_magcalib_status(&mag_calib_status);
            bno055_get_syscalib_status(&sys_calib_status);
            Serial2.print("Time Stamp: ");
            Serial2.println(last_time);
            Serial2.print("Magnetometer Calibration Status: ");
            Serial2.println(mag_calib_status);
            Serial2.print("System Calibration Status: ");
            Serial2.println(sys_calib_status);
            last_time = millis();
        }
    }
}

float invert_heading(float raw_h) {
    float real_h = raw_h - 90.0;
    if (real_h < 0) real_h += 360;
    return real_h;
}

// float correct_heading(float raw_heading, float offset) {
//     float corrected = raw_heading - offset;
//     if (corrected < 0) corrected += 360;
//     else if (corrected >= 360) corrected -= 360;
//     return corrected;
// }

float calc_yaw(float real_h) {
    return real_h > 180 ? real_h - 360.0f : real_h;
}

void extract_euler() {
    bno055_read_euler_hrp(&bno_euler);

    float raw_heading = ((float)bno_euler.h / EULER_TO_DEG);
    if (heading < 0) heading += 360;
    else if (heading >= 360) heading -= 360;

    roll = -(float)bno_euler.p / EULER_TO_DEG;
    pitch = (float)bno_euler.r / EULER_TO_DEG;

    yaw = calc_yaw(heading);
    
    rad_roll = (float)bno_euler.p / EULER_TO_RAD;
    rad_pitch = (float)bno_euler.r / EULER_TO_RAD;
    rad_yaw = radians(yaw);

    bno055_get_syscalib_status(&sys_calib_status);
    bno055_get_magcalib_status(&mag_calib_status);
    bno055_get_gyrocalib_status(&gyro_calib);
    bno055_get_accelcalib_status(&accel_calib);
}

void extract_gyroscope() {
    bno055_read_gyro_xyz(&bno_gyro);
    gyro_x = ((float)bno_gyro.x / GYRO_TO_DPS);
    gyro_y = -1 * ((float)bno_gyro.y / GYRO_TO_DPS);
    gyro_z = ((float)bno_gyro.z / GYRO_TO_DPS);
}

void extract_accelerometer() {
    bno055_read_accel_xyz(&bno_accel);
    _accel.x = (float)bno_accel.x;
    _accel.y = (float)bno_accel.y;
    _accel.z = (float)bno_accel.z;
}

void extract_linear_accelerometer() {
    bno055_read_linear_accel_xyz(&bno_linear);
    _linear_acc.x = (float)bno_linear.x / ACCEL_TO_MS2;
    _linear_acc.y = (float)bno_linear.y / ACCEL_TO_MS2;
    _linear_acc.z = (float)bno_linear.z / ACCEL_TO_MS2;
}

void init_imu() {
    Wire.begin(21, 22);  // Initialize I2C with SDA on GPIO 21 and SCL on GPIO 22
    Wire.setClock(400000);  // Set I2C clock speed to 400kHz
    I2Cdev::initialize();
    BNO_Init(&bno_data);
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(1 << 9);
}

void update_imu() {
    extract_linear_accelerometer();
    extract_euler();
    extract_gyroscope();
    extract_accelerometer();

    delta_roll = abs(roll - last_roll);
    delta_pitch = abs(pitch - last_pitch);
    delta_yaw = abs(yaw - last_yaw);

    last_roll = roll;
    last_pitch = pitch;
    last_yaw = yaw;
}