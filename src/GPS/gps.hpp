#ifndef GPS_WITH_IMU_GPS_GPS_HPP
#define GPS_WITH_IMU_GPS_GPS_HPP

#include <SoftwareSerial.h>
#define GPS_TX_PIN 1
#define GPS_RX_PIN 0

typedef struct {
  char gprmc_message[100];
  double latitude_in_degrees;
  double longitude_in_degrees;
  char latitude_direction;
  char longitude_direction;
  double latitude_in_ddmmss;
  double longitude_in_ddmmss;
  double altitude;
  double speed;
  double time;
  double date;
  bool gps_status;
} gps_data;

SoftwareSerial gps_serial_(GPS_RX_PIN, GPS_TX_PIN);
namespace gps_with_imu {
class gps {
 private:
  gps_data gps_data_;
  char latitude_string_[12], longitude_string_[13];
  char latitude_direction_, longitude_direction_;
  double latitude_in_degrees_, longitude_in_degrees_, latitude_in_radians_,
      longitude_in_radians_, latitude_in_ddmmss_, longitude_in_ddmmss,
      target_latitude_, target_longitude_;
  void get_gprmc_message(gps_data& gps_data_);
  void get_gps_status(gps_data& gps_data_);
  void get_speed(gps_data& gps_data_);
  void get_time(gps_data& gps_data_);
  void get_date(gps_data& gps_data_);
  void get_lat_long(gps_data& gps_data_);
  void get_lat_long_in_degrees(gps_data& gps_data_);
  void get_lat_long_in_radians(gps_data& gps_data_);

 public:
  gps(/* args */);
  ~gps();
  void initialize();
  void cold_start();
  void warm_start();
  void hot_start();
  void get_target_coordinates(double& target_latitude,
                              double& target_longitude);
  void get_gps_data(gps_data& gps_data_);
  void get_distance(double &distance, double latitude_in_degrees,
                  double longitude_in_degrees, double target_latitude,
                  double target_longitude);
  void getDirection(double& bearing);
};

}  // namespace gps_with_imu

#endif