#pragma once
#include <Arduino.h>
#include "header.h"
// #include "motor_control.h"

// Interrupt handlers for encoders
void IRAM_ATTR encoderM1()
{
  if (digitalRead(ENC_M1_B) == LOW)
    countM1_++;
  else
    countM1_--;
}

void IRAM_ATTR encoderM2()
{
  // if (digitalRead(ENC_M2_B) == LOW) countM2_++;

  // to fix the reverse increment issue for M2 encoder
  if (digitalRead(ENC_M2_B) == HIGH)
    countM2_++; // Swapped from LOW to HIGH

  else
    countM2_--;
}

void IRAM_ATTR encoderM3()
{
  if (digitalRead(ENC_M3_B) == LOW)
    countM3_++;
  else
    countM3_--;
}

void IRAM_ATTR encoderM4()
{
  if (digitalRead(ENC_M4_B) == LOW)
    countM4_++;
  else
    countM4_--;
}

// Motor control functions
void setupMotors()
{
  noInterrupts(); // Disable interrupts during setup

  // First disable all motors before anything else
  pinMode(EN_M1, OUTPUT);
  digitalWrite(EN_M1, LOW);
  pinMode(EN_M2, OUTPUT);
  digitalWrite(EN_M2, LOW);
  pinMode(EN_M3, OUTPUT);
  digitalWrite(EN_M3, LOW);
  pinMode(EN_M4, OUTPUT);
  digitalWrite(EN_M4, LOW);

  int motorPins[] = {M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2,
                     EN_M1, EN_M2, EN_M3, EN_M4};
  for (int i = 0; i < 12; i++)
  {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  int encoderPins[] = {ENC_M1_A, ENC_M1_B, ENC_M2_A, ENC_M2_B,
                       ENC_M3_A, ENC_M3_B, ENC_M4_A, ENC_M4_B};
  for (int i = 0; i < 8; i++)
  {
    pinMode(encoderPins[i], INPUT_PULLUP);
  }

  interrupts(); // Re-enable interrupts before attaching handlers

  // Wait a moment for pins to stabilize
  delay(10);

  attachInterrupt(digitalPinToInterrupt(ENC_M1_A), encoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_A), encoderM2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_A), encoderM3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M4_A), encoderM4, RISING);

  pid_right.set_output_limits(-255, 255);
  pid_left.set_output_limits(-255, 255);

  resetEncoders();

  last_encoder_reading_.M1 = countM1_;
  last_encoder_reading_.M1 = countM2_;
  last_encoder_reading_.M1 = countM3_;
  last_encoder_reading_.M1 = countM4_;
  last_data_reading_time_.M1 = millis();
  last_data_reading_time_.M2 = millis();
  last_data_reading_time_.M3 = millis();
  last_data_reading_time_.M4 = millis();
}

void setMotorDirection(bool forward)
{
  digitalWrite(M1_IN1, forward);
  digitalWrite(M1_IN2, !forward);
  digitalWrite(M2_IN1, forward);
  digitalWrite(M2_IN2, !forward);
  digitalWrite(M3_IN1, forward);
  digitalWrite(M3_IN2, !forward);
  digitalWrite(M4_IN1, forward);
  digitalWrite(M4_IN2, !forward);
}

void setTurnLeft()
{
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, HIGH);
  digitalWrite(M4_IN1, HIGH);
  digitalWrite(M4_IN2, LOW);
}

void setTurnRight()
{
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, HIGH);
  digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, HIGH);
}

void accelerateTo(int targetSpeed)
{
  for (int spd = 0; spd <= targetSpeed; spd++)
  {
    analogWrite(EN_M1, spd);
    analogWrite(EN_M2, spd);
    analogWrite(EN_M3, spd);
    analogWrite(EN_M4, spd);
    delay(STEP_DELAY);
  }
}

void decelerateFrom(int startSpeed)
{
  for (int spd = startSpeed; spd >= 0; spd--)
  {
    analogWrite(EN_M1, spd);
    analogWrite(EN_M2, spd);
    analogWrite(EN_M3, spd);
    analogWrite(EN_M4, spd);
    delay(STEP_DELAY);
  }
}

void moveForward(int left, int right)
{
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, HIGH);
  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, HIGH);

  analogWrite(EN_M1, right);
  analogWrite(EN_M2, left);
  analogWrite(EN_M3, right);
  analogWrite(EN_M4, left);
}

void moveBackward(int speed)
{
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, HIGH);
  digitalWrite(M3_IN1, HIGH);
  digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, HIGH);
  digitalWrite(M4_IN2, LOW);

  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);
}

void turnLeft(int left, int right)
{
  setTurnLeft();
  analogWrite(EN_M1, right);
  analogWrite(EN_M2, left);
  analogWrite(EN_M3, right);
  analogWrite(EN_M4, left);
}

void turnRight(int left, int right)
{
  setTurnRight();
  analogWrite(EN_M1, right);
  analogWrite(EN_M2, left);
  analogWrite(EN_M3, right);
  analogWrite(EN_M4, left);
}

void send_pwm(float left, float right) {
  analogWrite(EN_M1, left);
  analogWrite(EN_M2, right);
  analogWrite(EN_M3, left);
  analogWrite(EN_M4, right);
}

void moveStop()
{

  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, LOW);
  digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, LOW);
  digitalWrite(M4_IN2, LOW);
  analogWrite(EN_M1, 0);
  analogWrite(EN_M2, 0);
  analogWrite(EN_M3, 0);
  analogWrite(EN_M4, 0);
}

void compute_rpm()
{
  auto ticksl = countM1_;
  auto ticks2 = countM2_;
  auto ticks3 = countM3_;
  auto ticks4 = countM4_;

  auto now = millis();
  auto dt_time1 = now - last_data_reading_time_.M1;
  auto dt_time2 = now - last_data_reading_time_.M2;
  auto dt_time3 = now - last_data_reading_time_.M3;
  auto dt_time4 = now - last_data_reading_time_.M4;

  auto dt_ticks1 = ticksl - last_encoder_reading_.M1;
  auto dt_ticks2 = ticks2 - last_encoder_reading_.M2;
  auto dt_ticks3 = ticks3 - last_encoder_reading_.M3;
  auto dt_ticks4 = ticks4 - last_encoder_reading_.M4;

  rpm_.M1 = (float(dt_ticks1) / dt_time1) * 60000.0 / TICKS_PER_REV;
  rpm_.M2 = (float(dt_ticks2) / dt_time2) * 60000.0 / TICKS_PER_REV;
  rpm_.M3 = (float(dt_ticks3) / dt_time3) * 60000.0 / TICKS_PER_REV;
  rpm_.M4 = (float(dt_ticks4) / dt_time4) * 60000.0 / TICKS_PER_REV;

  last_encoder_reading_.M1 = countM1_;
  last_encoder_reading_.M2 = countM2_;
  last_encoder_reading_.M3 = countM3_;
  last_encoder_reading_.M4 = countM4_;

  last_data_reading_time_.M1 = now;
  last_data_reading_time_.M2 = now;
  last_data_reading_time_.M3 = now;
  last_data_reading_time_.M4 = now;

}

void compute_angular_velocity()
{
  angular_velocity_.M1 = rpm_.M1 * 2 * PI / 60;
  angular_velocity_.M2 = rpm_.M2 * 2 * PI / 60;
  angular_velocity_.M3 = rpm_.M3 * 2 * PI / 60;
  angular_velocity_.M4 = rpm_.M4 * 2 * PI / 60;
}

void compute_velocity()
{
  velocity_.M1 = angular_velocity_.M1 * WHEEL_RADIUS;
  velocity_.M2 = angular_velocity_.M2 * WHEEL_RADIUS;
  velocity_.M3 = angular_velocity_.M3 * WHEEL_RADIUS;
  velocity_.M4 = angular_velocity_.M4 * WHEEL_RADIUS;
}

void compute_distance()
{
  distance_.M1 = countM1_ * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  distance_.M2 = countM2_ * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  distance_.M3 = countM3_ * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
  distance_.M4 = countM4_ * 2 * PI * WHEEL_RADIUS / TICKS_PER_REV;
}

void compute_motor_data()
{
  compute_rpm();
  compute_angular_velocity();
  compute_velocity();
  compute_distance();
}

inline void bound(float &value, float min, float max)
{
  if (value < min)
  {
    value = min;
  }
  else if (value > max)
  {
    value = max;
  }
}

void resetEncoders()
{
  noInterrupts();
  countM1_ = 0;
  countM2_ = 0;
  countM3_ = 0;
  countM4_ = 0;
  last_encoder_reading_.M1 = countM1_;
  last_encoder_reading_.M2 = countM2_;
  last_encoder_reading_.M3 = countM3_;
  last_encoder_reading_.M4 = countM4_;
  interrupts();
}

void printEncoders()
{
  Serial.print("Encoders: ");
  Serial.print("M1: ");
  Serial.print(countM1_);
  Serial.print(" | M2: ");
  Serial.print(countM2_);
  Serial.print(" | M3: ");
  Serial.print(countM3_);
  Serial.print(" | M4: ");
  Serial.println(countM4_);
}
