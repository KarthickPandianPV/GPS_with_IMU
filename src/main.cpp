#include "main.hpp"

using namespace gps_with_imu;

void setup() {
  Serial.begin(9600);
  gps.initialize(gps_serial_);
  gps.get_target_coordinates(target_latitude, target_longitude);
  Serial.print("Target latitude : ");
  Serial.println(target_latitude);
  Serial.print("Target longitude : ");
  Serial.println(target_longitude);
  delay(1000);
}

void loop() {
  gps.get_gps_data(gps_serial_, gps_data_);
  if (gps_data_.gps_status) {
    Serial.println("GPS is locked.");
    gps.get_distance(target_distance, gps_data_.latitude_in_degrees,
                     gps_data_.longitude_in_degrees, target_latitude,
                     target_longitude);
    Serial.print("Distance to target : ");
    Serial.println(target_distance);
    gps.get_direction(target_bearing, gps_data_.latitude_in_degrees,
                      gps_data_.longitude_in_degrees, target_latitude,
                      target_longitude);
    Serial.print("Direction to target : ");
    Serial.println(target_bearing);
  } else {
    Serial.println("GPS is not locked.");
  }
}