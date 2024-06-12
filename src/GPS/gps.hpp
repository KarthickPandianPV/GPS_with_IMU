#ifndef GPS_WITH_IMU_GPS_GPS_HPP
#define GPS_WITH_IMU_GPS_GPS_HPP

#include <SoftwareSerial.h>
namespace gps_with_imu {
class gps {
 private:
  SoftwareSerial gps_serial_;
  char latitude_string_[12], longitude_string_[13];
  char latitude_direction_, longitude_direction_;
  double latitude_in_degrees_, longitude_in_degrees_, latitude_in_radians_,
      longitude_in_radians_, latitude_in_ddmmss_, longitude_in_ddmmss,
      target_latitude_, target_longitude_;

 public:
  gps(/* args */);
  ~gps();

  void initialize();
  void cold_start();
  void warm_start();
  void hot_start();
};

}  // namespace gps_with_imu

#endif