#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

#include "GPS/gps.hpp"
#include "IMU/imu.hpp"
#include "communication/communication.hpp"
#include "vec3f.hpp"

#define GPS_TX_PIN 1
#define GPS_RX_PIN 0
using namespace gps_with_imu;

int number_of_samples = 3000;
vec3f acceleration;
vec3f magnetic_field = {0, 0, 0};
vec3f velocity = {0, 0, 0}, distance = {0, 0, 0};
double initial_time = 0, delta_time = 0;
double target_latitude = 15.35, target_longitude = 73.68, target_distance,
       target_bearing;

extern SoftwareSerial gps_serial_;
extern double target_latitude, target_longitude, target_distance,
    target_bearing;
extern Gps gps;
extern Imu imu;

#endif  // MAIN_HPP
