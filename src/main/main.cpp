#include "main.hpp"

// SoftwareSerial gps_serial_(GPS_RX_PIN, GPS_TX_PIN);
// Gps gps;
Imu imu;
void setup() {
  // Serial.begin(9600);
  // while (!Serial) {
  //   Serial.println("Serial not available.");
  //   delay(1);
  // }
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  // gps.initialize(gps_serial_);
  // // gps.get_target_coordinates(target_latitude, target_longitude);
  // Serial.print("Target latitude : ");
  // Serial.println(target_latitude);
  // Serial.print("Target longitude : ");
  // Serial.println(target_longitude);
  // delay(1000);
  initializeCommunication();
  imu.MPUinitialize();
  imu.initializeAhrs();
  imu.MPUcalculateOffsets(number_of_samples);
}

void loop() {
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
  imu.MPUupdateReadings();
  imu.calibrateImu();
  imu.updateOrientation();
  acceleration = imu.getAcceleration();
  angular_velocity = imu.getAngularVelocity();
  magnetic_field = imu.getMagneticField();
  earth_acceleration = imu.getEarthAcceleration();
  euler_orientation = imu.getEulerOrientation();
  sendData(acceleration, angular_velocity, magnetic_field, earth_acceleration,
           euler_orientation);
  checkForCommands();
  // Serial.print("   ax: ");
  // Serial.print(acceleration.x, 4);
  // Serial.print("   ay: ");
  // Serial.print(acceleration.y, 5);
  // Serial.print("   az: ");
  // Serial.print(acceleration.z, 5);

  // Serial.print("   gx: ");
  // Serial.print(angular_velocity.x, 4);
  // Serial.print("   gy: ");
  // Serial.print(angular_velocity.y, 5);
  // Serial.print("   gz: ");
  // Serial.print(angular_velocity.z, 5);

  // Serial.print("   mx: ");
  // Serial.print(magnetic_field.x, 4);
  // Serial.print("   my: ");
  // Serial.print(magnetic_field.y, 5);
  // Serial.print("   mz: ");
  // Serial.print(magnetic_field.z, 5);

  // Serial.print("   ax': ");
  // Serial.print(earth_acceleration.x, 4);
  // Serial.print("   ay': ");
  // Serial.print(earth_acceleration.y, 5);
  // Serial.print("   az': ");
  // Serial.println(earth_acceleration.z, 5);

  // Serial.print("   roll: ");
  // Serial.print(euler_orientation.x);
  // Serial.print("   pitch: ");
  // Serial.print(euler_orientation.y);
  // Serial.print("   yaw");
  // Serial.println(euler_orientation.z);
  delay(10);
}

//  if ((acceleration.x <= 0.1) && (acceleration.x >= -0.1)) {
//     acceleration.x = 0;
//   }
//   if ((acceleration.y <= 0.1) && (acceleration.y >= -0.1)) {
//     acceleration.y = 0;
//   }
//   if ((acceleration.z <= 0.1) && (acceleration.z >= -0.1)) {
//     acceleration.z = 0;
//   }
//   if ((magnetic_field.x <= 0.1) && (magnetic_field.x >= -0.1)) {
//     magnetic_field.x = 0;
//   }
//   if ((magnetic_field.y <= 0.1) && (magnetic_field.y >= -0.1)) {
//     magnetic_field.y = 0;
//   }
//   if ((magnetic_field.z <= 0.1) && (magnetic_field.z >= -0.1)) {
//     magnetic_field.z = 0;
//   }