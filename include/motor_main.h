#pragma once
#include <Arduino.h>

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

// #define COUNTS_PER_REV 100 // 100 pulses per revolution with 4x decoding
// #define WHEEL_RADIUS 0.035 // in meters (30mm)
// #define WHEEL_BASE 0.145 // in meters (distance between left and right wheels) 14,5cm

// #define TURN_90_COUNTS ((WHEEL_BASE) / (4 * WHEEL_RADIUS * 2) * COUNTS_PER_REV)

// Encoder counts
volatile long countM1 = 0;
volatile long countM2 = 0;
volatile long countM3 = 0;
volatile long countM4 = 0;

// Interrupt handlers for encoders
void IRAM_ATTR encoderM1() {
  if (digitalRead(ENC_M1_B) == LOW) countM1++;
  else countM1--;
}

void IRAM_ATTR encoderM2() {
  // if (digitalRead(ENC_M2_B) == LOW) countM2++;

  // to fix the reverse increment issue for M2 encoder
  if (digitalRead(ENC_M2_B) == HIGH) countM2++; // Swapped from LOW to HIGH

  else countM2--;
}

void IRAM_ATTR encoderM3() {
  if (digitalRead(ENC_M3_B) == LOW) countM3++;
  else countM3--;
}

void IRAM_ATTR encoderM4() {
  if (digitalRead(ENC_M4_B) == LOW) countM4++;
  else countM4--;
}

// Motor control functions
void setupMotors() {
  noInterrupts();  // Disable interrupts during setup


  // First disable all motors before anything else
  pinMode(EN_M1, OUTPUT); digitalWrite(EN_M1, LOW);
  pinMode(EN_M2, OUTPUT); digitalWrite(EN_M2, LOW);
  pinMode(EN_M3, OUTPUT); digitalWrite(EN_M3, LOW);
  pinMode(EN_M4, OUTPUT); digitalWrite(EN_M4, LOW);

  int motorPins[] = {M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2,
                     EN_M1, EN_M2, EN_M3, EN_M4};
  for (int i = 0; i < 12; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  int encoderPins[] = {ENC_M1_A, ENC_M1_B, ENC_M2_A, ENC_M2_B, 
                      ENC_M3_A, ENC_M3_B, ENC_M4_A, ENC_M4_B};
  for (int i = 0; i < 8; i++) {
    pinMode(encoderPins[i], INPUT_PULLUP);
  }

  interrupts();  // Re-enable interrupts before attaching handlers


  // Wait a moment for pins to stabilize
  delay(10);

  attachInterrupt(digitalPinToInterrupt(ENC_M1_A), encoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_A), encoderM2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_A), encoderM3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M4_A), encoderM4, RISING);
}

void setMotorDirection(bool forward) {
  digitalWrite(M1_IN1, forward); digitalWrite(M1_IN2, !forward);
  digitalWrite(M2_IN1, forward); digitalWrite(M2_IN2, !forward);
  digitalWrite(M3_IN1, forward); digitalWrite(M3_IN2, !forward);
  digitalWrite(M4_IN1, forward); digitalWrite(M4_IN2, !forward);
}

void setTurnLeft() {
  digitalWrite(M1_IN1, HIGH);  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH);
  digitalWrite(M3_IN1, LOW);  digitalWrite(M3_IN2, HIGH);
  digitalWrite(M4_IN1, HIGH); digitalWrite(M4_IN2, LOW);
}

void setTurnRight() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, HIGH);  digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, LOW);  digitalWrite(M4_IN2, HIGH);
}

void accelerateTo(int targetSpeed) {
  for (int spd = 0; spd <= targetSpeed; spd++) {
    analogWrite(EN_M1, spd);
    analogWrite(EN_M2, spd);
    analogWrite(EN_M3, spd);
    analogWrite(EN_M4, spd);
    delay(STEP_DELAY);
  }
}

void decelerateFrom(int startSpeed) {
  for (int spd = startSpeed; spd >= 0; spd--) {
    analogWrite(EN_M1, spd);
    analogWrite(EN_M2, spd);
    analogWrite(EN_M3, spd);
    analogWrite(EN_M4, spd);
    delay(STEP_DELAY);
  }
}

void moveForward(int speed) {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, HIGH);
  digitalWrite(M4_IN1, LOW); digitalWrite(M4_IN2, HIGH);
  
  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);
}

void moveBackward(int speed) {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH);
  digitalWrite(M3_IN1, HIGH); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, HIGH); digitalWrite(M4_IN2, LOW);
  
  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);
}

void turnLeft(int speed) {
  setTurnLeft();
  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);
}

void turnRight(int speed) {
  setTurnRight();
  analogWrite(EN_M1, speed);
  analogWrite(EN_M2, speed);
  analogWrite(EN_M3, speed);
  analogWrite(EN_M4, speed);
}

void moveStop() {

  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, LOW); digitalWrite(M3_IN2, LOW);
  digitalWrite(M4_IN1, LOW); digitalWrite(M4_IN2, LOW);
  analogWrite(EN_M1, 0);
  analogWrite(EN_M2, 0);
  analogWrite(EN_M3, 0);
  analogWrite(EN_M4, 0);
}

void resetEncoders() {
  noInterrupts();
  countM1 = 0;
  countM2 = 0;
  countM3 = 0;
  countM4 = 0;
  interrupts();
}

void printEncoders() {
  Serial.print("Encoders: ");
  Serial.print("M1: "); Serial.print(countM1);
  Serial.print(" | M2: "); Serial.print(countM2);
  Serial.print(" | M3: "); Serial.print(countM3);
  Serial.print(" | M4: "); Serial.println(countM4);
}
