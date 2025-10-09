#pragma once
#include <Arduino.h>
#include "header.h"
#include "motor_driver.h"

void update_pid_gain()
{
  // MOTOR_DRIVER_PID_KP = 0;
  // MOTOR_DRIVER_PID_KI = 0;
  // MOTOR_DRIVER_PID_KD = 0;
  pid_right.set_pid_gains(MOTOR_DRIVER_PID_KP, MOTOR_DRIVER_PID_KI, MOTOR_DRIVER_PID_KD);
  pid_left.set_pid_gains(MOTOR_DRIVER_PID_KP, MOTOR_DRIVER_PID_KI, MOTOR_DRIVER_PID_KD);
}

void compute_wheel_speeds_()
{
  float v_r = (2 * cmd_vel_.x + cmd_vel_.w * WHEEL_BASE) /
              (2 * WHEEL_RADIUS);
  float v_l = (2 * cmd_vel_.x - cmd_vel_.w * WHEEL_BASE) /
              (2 * WHEEL_RADIUS);

  // Downscale velocity for motor max velocity
  // v_r = v_r * 0.0338;
  // v_l = v_l * 0.0338;

  bound(v_r, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY);
  bound(v_l, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY);

  pid_right.set_setpoint(v_r);
  pid_left.set_setpoint(v_l);
}

inline float apply_deadband(float pwm)
{
  if (fabs(pwm) < 1e-3f)
  {
    return 0.0f;
  }

  const float sign = pwm > 0.0f ? 1.0f : -1.0f;
  float magnitude = fabs(pwm);

  magnitude = MOTOR_PWM_DEADBAND + (magnitude / MOTOR_PWM_MAX) * (MOTOR_PWM_MAX - MOTOR_PWM_DEADBAND);

  if (magnitude > MOTOR_PWM_MAX)
  {
    magnitude = MOTOR_PWM_MAX;
  }
  else if (magnitude < MOTOR_PWM_DEADBAND)
  {
    magnitude = MOTOR_PWM_DEADBAND;
  }

  return sign * magnitude;
}

void motor_loop()
{
  compute_motor_data();
  auto error_motor_right = pid_right.compute(velocity_.M1);
  auto error_motor_left = pid_left.compute(velocity_.M2);

  bound(error_motor_right, -255, 255);
  bound(error_motor_left, -255, 255);

  auto pwm_motor_right = apply_deadband(error_motor_right);
  auto pwm_motor_left = apply_deadband(error_motor_left);

  if (pwm_motor_right < 0) {
    setTurnLeft();
  }
  else if (pwm_motor_right > 0) {
    setTurnRight();
  }
  else {
    moveStop();
  }

  send_pwm(pwm_motor_left, pwm_motor_right);
}
