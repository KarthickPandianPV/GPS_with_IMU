#include "main.hpp"

SoftwareSerial gps_serial_(GPS_RX_PIN, GPS_TX_PIN);
Gps gps;
gps_data gps_data_;
Imu imu;

void setup() {
  Serial.begin(9600);
  // gps.initialize(gps_serial_);
  // // gps.get_target_coordinates(target_latitude, target_longitude);
  // Serial.print("Target latitude : ");
  // Serial.println(target_latitude);
  // Serial.print("Target longitude : ");
  // Serial.println(target_longitude);
  // delay(1000);
  imu.initialize();
  imu.calculateOffsets(number_of_samples);
}

void loop() {
  long initial = millis();
  // gps.get_gps_data(gps_serial_);
  // if (gps_data_.gps_status) {
  //   Serial.println("GPS is locked.");
  //   gps.get_distance(target_distance, target_latitude, target_longitude);
  //   Serial.print("Distance to target : ");
  //   Serial.println(target_distance);
  //   gps.get_direction(target_bearing, target_latitude, target_longitude);
  //   Serial.print("Direction to target : ");
  //   Serial.println(target_bearing);
  // } else {
  //   Serial.println("GPS is not locked..!");
  // }
  // Serial.println(gps_data_.gprmc_message);
  imu.UpdateReadings();
  acceleration = imu.calibrateAccelerometer();
  magnetic_field = imu.calibrateMagnetometer();
  if ((acceleration.y <= 0.3) && (acceleration.y >= -0.3)) {
    acceleration.y = 0;
    velocity.y = 0;
  }
  double delta = millis() - initial;
  delta /= 1000;
  velocity.y += acceleration.y * delta;
  distance.y += (velocity.y * delta);
  Serial.print("ay: ");
  Serial.print(acceleration.y, 4);
  Serial.print("   sy: ");
  Serial.print(distance.y, 5);
  Serial.print("   vy: ");
  Serial.println(velocity.y, 5);
  // delay(1);
}

// Serial.print("ax: ");
// Serial.print(acceleration.x, 6);
// Serial.print("   ");
// Serial.print("ay: ");
// Serial.print(accelerxcation.y, 6);
// Serial.print("   ");
// Serial.print("az: ");
// Serial.print(acceleration.z, 6);
// Serial.println("   ");
// Serial.print("mx: ");
// Serial.print(magnetic_field.x, 6);
// Serial.print("   ");
// Serial.print("my: ");
// Serial.print(magnetic_field.y, 6);
// Serial.print("   ");
// Serial.print("mz: ");
// Serial.print(magnetic_field.z, 6);
// Serial.println("   ");
// delay(100);