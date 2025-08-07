#include <Arduino.h>
#include <firmware_control.h>

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
  // Serial.begin(115200);
  // delay(150); // Pastikan serial siap
  // init_imu(); // Inisialisasi sensor BNO055
  // Serial.println("BNO055 initialized");
  setupFirmware();

}

void loop() {
  // // update_imu();
  // extract_heading(); // Baca heading dari sensor
  // printUSB();
  // delay(100);  // Delay for readability
  loopFirmware();

  
}

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }