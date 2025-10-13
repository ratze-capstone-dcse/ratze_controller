#include <Arduino.h>
#include <WiFi.h>
#include <firmware_control.h>

// WiFi credentials
// const char* ssid = "Ratze";
// const char* password = "1010101010";

// void setupWiFi() {
//   WiFi.begin(ssid, password);
//   Serial.print("Connecting to WiFi");
  
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
  
//   Serial.println();
//   Serial.println("WiFi connected!");
//   Serial.print("IP address: ");
//   Serial.println(WiFi.localIP());
// }

void printUSB() {
  // Serial.println("Hello, USB!");
  Serial.print("Current millis: ");
  Serial.print(millis());
  // Serial.print(" Roll: ");
  // Serial.print(roll);
  // Serial.print(" Pitch: ");
  // Serial.print(pitch);
  // Serial.print(" Yaw: ");
  // Serial.print(yaw);
  // Serial.print(" Heading: ");
  // Serial.print(heading);
  Serial.print("  Heading: ");
  Serial.print(corrected_heading, 2);
  Serial.print(" | Kalibrasi (SYS-G-A-M): ");
  Serial.print(sys_calib);
  Serial.print("-");
  Serial.print(gyro_calib);
  Serial.print("-");
  Serial.print(accel_calib);
  Serial.print("-");
  Serial.println(mag_calib);
  // Serial.print(" Gyro X: ");
  // Serial.print(gxrs);
  // Serial.print(" Gyro Y: ");
  // Serial.print(gyrs);
  // Serial.print(" Gyro Z: ");
  // Serial.print(gzrs);
  // Serial.print(" Vertical Velocity: ");
  // Serial.println(vertical_velocity);
  // Serial.print(" Roll Error: ");
  // Serial.print(roll_error);
  // Serial.print(" Pitch Error: ");
  // Serial.println(pitch_error);
  // Serial.print("Yaw SP: ");
  // Serial.println(yaw_sp);
  // Serial.print("Last Yaw: ");
  // Serial.println(last_yaw);
  // Serial.print(" Acceleration Z: ");
  // Serial.println(accelData.z);
}



void setup() {
  Serial.begin(115200);
  delay(150);
  
  // setupWiFi();
  setupFirmware();

}

void loop() {
  // Check WiFi connection status
  // if (WiFi.status() != WL_CONNECTED) {
  //   Serial.println("WiFi disconnected, attempting reconnection...");
  //   setupWiFi();
  // }
  
  loopFirmware();

  
}

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }