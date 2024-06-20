#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

#include "GPS/gps.hpp"
#include "IMU/imu.hpp"
#include "vec3f.hpp"

#define GPS_TX_PIN 1
#define GPS_RX_PIN 0
using namespace gps_with_imu;

extern SoftwareSerial gps_serial_;
extern double target_latitude, target_longitude, target_distance,
    target_bearing;
extern Gps gps;

#endif  // MAIN_HPP
