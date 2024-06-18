#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

#include "GPS/gps.hpp"

#define GPS_TX_PIN 1
#define GPS_RX_PIN 0
using namespace gps_with_imu;

extern SoftwareSerial gps_serial_(GPS_RX_PIN, GPS_TX_PIN);
extern double target_latitude, target_longitude, target_distance,
    target_bearing;
extern Gps gps;
extern gps_data gps_data_;

#endif  // MAIN_HPP
