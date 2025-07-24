#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <BNO055_support.h>

// BNO055 Operation Modes
#define OPERATION_MODE_CONFIG      0x00
#define OPERATION_MODE_COMPASS     0x09

#define EULER_TO_DEG               16

// Data struktur
struct Vector3f {
    float x, y, z;
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

// Global data
float heading = 0.0;
float heading_offset = 0.0;
float corrected_heading = 0.0;
unsigned char sys_calib = 0, gyro_calib = 0, accel_calib = 0, mag_calib = 0;

struct bno055_t bno_data;
struct bno055_euler bno_euler;

// Inisialisasi IMU dengan mode kompas
void init_imu() {
    Wire.begin(21, 22);            // SDA = 21, SCL = 22 (ESP32)
    Wire.setClock(400000);         // I2C Fast Mode
    I2Cdev::initialize();
    BNO_Init(&bno_data);

    bno055_set_operation_mode(OPERATION_MODE_CONFIG); // Masuk mode konfigurasi
    delay(25);

    bno055_set_operation_mode(OPERATION_MODE_COMPASS); // Mode kompas absolut
    delay(600); // Delay untuk stabilisasi sensor

    Serial.println("IMU initialized in COMPASS mode");
}

// Baca heading absolut dari sensor
void extract_heading() {
    bno055_read_euler_hrp(&bno_euler);
    heading = ((float)bno_euler.h) / EULER_TO_DEG;

    corrected_heading = heading - heading_offset + 90;
    if (corrected_heading < 0) corrected_heading += 360;
    else if (corrected_heading >= 360) corrected_heading -= 360;    

    // // Koreksi agar tetap di rentang 0–360
    // if (heading < 0) heading += 360;
    // else if (heading >= 360) heading -= 360;

    // Cek status kalibrasi
    bno055_get_syscalib_status(&sys_calib);
    bno055_get_magcalib_status(&mag_calib);
}
