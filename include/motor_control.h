#pragma once
#include <Arduino.h>
#include "header.h"
#include "motor_driver.h"

void update_pid_gain()
{
  pid_right.set_pid_gains(MOTOR_DRIVER_PID_KP, MOTOR_DRIVER_PID_KI, MOTOR_DRIVER_PID_KD);
  pid_left.set_pid_gains(MOTOR_DRIVER_PID_KP, MOTOR_DRIVER_PID_KI, MOTOR_DRIVER_PID_KD);
}

void compute_wheel_speeds_()
{
  float v_r = (2 * cmd_vel_.x + cmd_vel_.w * WHEEL_BASE) /
              (2 * WHEEL_RADIUS);
  float v_l = (2 * cmd_vel_.x - cmd_vel_.w * WHEEL_BASE) /
              (2 * WHEEL_RADIUS);

  // Serial.println("Wheel Speeds (rad/s): Right: " + String(v_r, 2) + " | Left: " + String(v_l, 2));

  // Downscale velocity for motor max velocity
  v_r = v_r * 0.0338;
  v_l = v_l * 0.0338;

  bound(v_r, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY);
  bound(v_l, -MOTOR_MAX_VELOCITY, MOTOR_MAX_VELOCITY);

  pid_right.set_setpoint(v_r);
  pid_left.set_setpoint(v_l);
}

inline float apply_deadband(float pwm)
{
  if (fabs(pwm) < 50.0f)
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
  compute_wheel_speeds_();

  compute_motor_data();
  auto error_motor_right = pid_right.compute(velocity_.M1);
  auto error_motor_left = pid_left.compute(velocity_.M2);

  bound(error_motor_right, -255, 255);
  bound(error_motor_left, -255, 255);

  // auto pwm_motor_right = static_cast<int>(apply_deadband(error_motor_right));
  // auto pwm_motor_left = static_cast<int>(apply_deadband(error_motor_left));

  auto pwm_motor_right = static_cast<int>(error_motor_right);
  auto pwm_motor_left = static_cast<int>(error_motor_left);

  if (pwm_motor_right > 0 && pwm_motor_left > 0)
  {
    moveForward(abs(pwm_motor_left), abs(pwm_motor_right));
  }
  else if (pwm_motor_right < 0 && pwm_motor_left > 0)
  {
    turnRight(abs(pwm_motor_left), abs(pwm_motor_right));
  }
  else if (pwm_motor_right > 0 && pwm_motor_left < 0)
  {
    turnLeft(abs(pwm_motor_left), abs(pwm_motor_right));
  }
  else if (pwm_motor_right < 0 && pwm_motor_left < 0)
  {
    moveBackward(min(abs(pwm_motor_right), abs(pwm_motor_left)));
  }
  else
  {
    moveStop();
  }

  Serial.println("PWM Right: " + String(pwm_motor_right) + " | PWM Left: " + String(pwm_motor_left));
  // send_pwm(pwm_motor_left, pwm_motor_right});
}
