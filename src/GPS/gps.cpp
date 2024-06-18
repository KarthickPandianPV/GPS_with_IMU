#include "gps.hpp"

#include <Arduino.h>

namespace gps_with_imu {
Gps::Gps() {}
Gps::~Gps() {}

double to_radians(double degrees) { return degrees * PI / 180.0; }

void Gps::warm_start(SoftwareSerial &gps_serial_) {
  Serial.println("Warming up the GPS module...");
  gps_serial_.println("$PSTMWARM");
  delay(20000);
  Serial.println("Warm start done.");
}

void Gps::initialize(SoftwareSerial &gps_serial_) {
  gps_serial_.begin(9600);
  delay(1000);
  warm_start(gps_serial_);
}

void Gps::get_target_coordinates(double &target_latitude,
                                 double &target_longitude) {
  Serial.print("Enter target latitude : ");
  target_latitude = Serial.parseFloat();
  Serial.print("Enter target longitude : ");
  target_longitude = Serial.parseFloat();
}

void Gps::get_gprmc_message(SoftwareSerial &gps_serial_, gps_data &gps_data_) {
  String recieved_message;
  int i = 0;
  while (gps_serial_.available() > 0) {
    recieved_message = gps_serial_.readStringUntil('\n');
  }
  if ((recieved_message[1] == 'G') && (recieved_message[2] == 'P') &&
      (recieved_message[3] == 'R') && (recieved_message[4] == 'M') &&
      (recieved_message[5] == 'C')) {
    while (recieved_message[i] != '\0') {
      gps_data_.gprmc_message[i] = recieved_message[i];
      i++;
    }
  }
}
void Gps::get_gps_status(gps_data &gps_data_) {
  if (gps_data_.gprmc_message[18] == 'A') {
    gps_data_.gps_status = true;
  } else {
    gps_data_.gps_status = false;
  }
}
void Gps::get_time(gps_data &gps_data_) {
  char time_string[10];
  int i;
  for (i = 7; i <= 16; i++) {
    time_string[i - 7] = gps_data_.gprmc_message[i];
  }
  gps_data_.time = atof(time_string);
}
void Gps::get_date(gps_data &gps_data_) {
  char date_string[6];
  int i;
  for (i = 57; i <= 62; i++) {
    date_string[i - 57] = gps_data_.gprmc_message[i];
    gps_data_.date = atof(date_string);
  }
}
void Gps::get_speed(gps_data &gps_data_) {
  char speed_string[3];
  int i;
  for (i = 47; i <= 49; i++) {
    speed_string[i - 47] = gps_data_.gprmc_message[i];
  }
  gps_data_.speed = atof(speed_string);
}
void Gps::get_lat_long_in_degrees(gps_data &gps_data_) {
  gps_data_.latitude_in_degrees = (int)gps_data_.latitude_in_ddmmss / 100;
  gps_data_.longitude_in_degrees = (int)gps_data_.longitude_in_ddmmss / 100;
  gps_data_.latitude_in_degrees =
      gps_data_.latitude_in_degrees +
      (gps_data_.latitude_in_ddmmss - 100.00 * gps_data_.latitude_in_degrees) /
          60.00;
  gps_data_.longitude_in_degrees = gps_data_.longitude_in_degrees +
                                   (gps_data_.longitude_in_ddmmss -
                                    100.00 * gps_data_.longitude_in_degrees) /
                                       60.00;
  if (gps_data_.latitude_direction == 'S') {
    gps_data_.latitude_in_degrees = -gps_data_.latitude_in_degrees;
  }
  if (gps_data_.longitude_direction == 'W') {
    gps_data_.longitude_in_degrees = -gps_data_.longitude_in_degrees;
  }
}
void Gps::get_lat_long(gps_data &gps_data_) {
  int i = 0, j = 0;
  char latitude_string[12], longitude_string[13];
  char latitude_direction, longitude_direction;
  double latitude, longitude;
  if (gps_data_.gps_status == 'A') {
    for (i = 20; i <= 29; i++) {
      latitude_string[i - 20] = gps_data_.gprmc_message[i];
    }
    latitude_direction = gps_data_.gprmc_message[31];
    for (j = 33; j <= 43; j++) {
      longitude_string[j - 33] = gps_data_.gprmc_message[j];
    }
    longitude_direction = gps_data_.gprmc_message[45];
    latitude = atof(latitude_string);
    longitude = atof(longitude_string);
    gps_data_.latitude_in_ddmmss = latitude;
    gps_data_.longitude_in_ddmmss = longitude;
    gps_data_.latitude_direction = latitude_direction;
    gps_data_.longitude_direction = longitude_direction;
  } else {
    gps_data_.gps_status = false;
    gps_data_.latitude_in_ddmmss = 0.0;
    gps_data_.longitude_in_ddmmss = 0.0;
    gps_data_.latitude_direction = 'N';
    gps_data_.longitude_direction = 'E';
    gps_data_.latitude_in_degrees = 0.0;
    gps_data_.longitude_in_degrees = 0.0;
  }
  get_lat_long_in_degrees(gps_data_);
}

void Gps::get_gps_data(SoftwareSerial &gps_serial_, gps_data &gps_data_) {
  get_gprmc_message(gps_serial_, gps_data_);
  get_time(gps_data_);
  get_date(gps_data_);
  get_speed(gps_data_);
  get_lat_long(gps_data_);
}
void Gps::get_distance(double &distance, double latitude_in_degrees,
                       double longitude_in_degrees, double target_latitude,
                       double target_longitude) {
  double latitude_in_radians, longitude_in_radians, target_latitude_radians,
      target_longitude_radians, delta_latitude, delta_longitude;
  latitude_in_radians = to_radians(latitude_in_degrees);
  longitude_in_radians = to_radians(longitude_in_degrees);
  target_latitude_radians = to_radians(target_latitude);
  target_longitude_radians = to_radians(target_longitude);
  delta_latitude = target_latitude_radians - latitude_in_radians;
  delta_longitude = target_longitude_radians - longitude_in_radians;
  distance =
      6371.01 * acos(sin(latitude_in_radians) * sin(target_latitude_radians) +
                     cos(latitude_in_radians) * cos(target_latitude_radians) *
                         cos(delta_longitude));
}

void Gps::get_direction(double &direction, double latitude_in_degrees,
                        double longitude_in_degrees, double target_latitude,
                        double target_longitude) {
  double latitude_in_radians, longitude_in_radians, target_latitude_radians,
      target_longitude_radians, delta_longitude;
  latitude_in_radians = to_radians(latitude_in_degrees);
  longitude_in_radians = to_radians(longitude_in_degrees);
  target_latitude_radians = to_radians(target_latitude);
  target_longitude_radians = to_radians(target_longitude);
  delta_longitude = target_longitude_radians - longitude_in_radians;
  direction =
      atan2(sin(delta_longitude) * cos(target_latitude_radians),
            cos(latitude_in_radians) * sin(target_latitude_radians) -
                sin(latitude_in_radians) * cos(target_latitude_radians) *
                    cos(delta_longitude));
  direction = direction * 180.0 / PI;
  direction = fmod((direction + 360.0), 360.0);
}

}  // namespace gps_with_imu