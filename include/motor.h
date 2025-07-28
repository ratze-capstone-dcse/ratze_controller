#include <Arduino.h>

#define M1_IN1 13
#define M1_IN2 12
#define M2_IN1 14
#define M2_IN2 27
#define M3_IN1 26
#define M3_IN2 25
#define M4_IN1 33
#define M4_IN2 32


#define EN_M1 18
#define EN_M2 19
#define EN_M3 22
#define EN_M4 21

#define ENC_M1_A 34
#define ENC_M1_B 35
#define ENC_M2_A 4
#define ENC_M2_B 15
#define ENC_M3_A 16
#define ENC_M3_B 17
#define ENC_M4_A 5
#define ENC_M4_B 18

#define MAX_SPEED 255
#define STEP_DELAY 5    

volatile long countM1 = 0;
volatile long countM2 = 0;
volatile long countM3 = 0;
volatile long countM4 = 0;

void IRAM_ATTR encoderM1() {
  if (digitalRead(ENC_M1_B) == LOW) countM1++;
  else countM1--;
}

void IRAM_ATTR encoderM2() {
  if (digitalRead(ENC_M2_B) == LOW) countM2++;
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

void setupPWM() {
}

void setMotorDirection(bool forward) {
  digitalWrite(M1_IN1, forward); digitalWrite(M1_IN2, !forward);
  digitalWrite(M2_IN1, forward); digitalWrite(M2_IN2, !forward);
  digitalWrite(M3_IN1, forward); digitalWrite(M3_IN2, !forward);
  digitalWrite(M4_IN1, forward); digitalWrite(M4_IN2, !forward);
}

void setTurnLeft() {
  digitalWrite(M1_IN1, LOW);  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
  digitalWrite(M3_IN1, LOW);  digitalWrite(M3_IN2, HIGH);
  digitalWrite(M4_IN1, HIGH); digitalWrite(M4_IN2, LOW);
}

void setTurnRight() {
  digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW);  digitalWrite(M2_IN2, HIGH);
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

void resetEncoders() {
  countM1 = 0;
  countM2 = 0;
  countM3 = 0;
  countM4 = 0;
}

void printEncoders() {
  Serial.print("Encoders: ");
  Serial.print("M1: "); Serial.print(countM1);
  Serial.print(" | M2: "); Serial.print(countM2);
  Serial.print(" | M3: "); Serial.print(countM3);
  Serial.print(" | M4: "); Serial.println(countM4);
}

void setup() {
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

  attachInterrupt(digitalPinToInterrupt(ENC_M1_A), encoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_A), encoderM2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_A), encoderM3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M4_A), encoderM4, RISING);

  Serial.begin(115200);
  Serial.println("Motor control with gradual acceleration/deceleration");
}

void loop() {
  resetEncoders();
  Serial.println("Moving forward...");
  setMotorDirection(true);
  accelerateTo(MAX_SPEED);
  delay(3000);
  decelerateFrom(MAX_SPEED);
  printEncoders();
  delay(1000);

  resetEncoders();
  Serial.println("Moving backward...");
  setMotorDirection(false);
  accelerateTo(MAX_SPEED);
  delay(3000);
  decelerateFrom(MAX_SPEED);
  printEncoders();
  delay(1000);

  resetEncoders();
  Serial.println("Turning left...");
  setTurnLeft();
  accelerateTo(MAX_SPEED);
  delay(3000);
  decelerateFrom(MAX_SPEED);
  printEncoders();
  delay(1000);

  resetEncoders();
  Serial.println("Turning right...");
  setTurnRight();
  accelerateTo(MAX_SPEED);
  delay(3000);
  decelerateFrom(MAX_SPEED);
  printEncoders();
  delay(3000);
}