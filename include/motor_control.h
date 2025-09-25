#pragma once
#include <Arduino.h>
#include "header.h"
#include "motor_driver.h"

void update_pid_gain() {
  // MOTOR_DRIVER_PID_KP = 0;
  // MOTOR_DRIVER_PID_KI = 0;
  // MOTOR_DRIVER_PID_KD = 0;
  pid_.set_pid_gains(MOTOR_DRIVER_PID_KP, MOTOR_DRIVER_PID_KI, MOTOR_DRIVER_PID_KD);
}

void set_velocity(float velocity) {
    bound(velocity, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY);
    pid_.set_setpoint(velocity);
}

void compute_wheel_speeds_() {
    float v_r = (2 * cmd_vel_.x + cmd_vel_.w * WHEEL_BASE) /
                (2 * WHEEL_RADIUS);
    float v_l = (2 * cmd_vel_.x - cmd_vel_.w * WHEEL_BASE) /
                (2 * WHEEL_RADIUS);

    // Downscale velocity for motor max velocity
    // v_r = v_r * 0.0338;
    // v_l = v_l * 0.0338;

    set_velocity(v_r);
    set_velocity(v_l);
}

void motor_loop() {
  compute_motor_data();
  auto error_motor1 = pid_.compute(velocity_.M1);
  auto error_motor2 = pid_.compute(velocity_.M2);
  auto error_motor3 = pid_.compute(velocity_.M3);
  auto error_motor4 = pid_.compute(velocity_.M4);

  bound(error_motor1, -255, 255);
  bound(error_motor2, -255, 255);
  bound(error_motor3, -255, 255);
  bound(error_motor4, -255, 255);

  if 

}

