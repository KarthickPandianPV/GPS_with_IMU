#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

#include "IMU/imu.hpp"
#include "communication/communication.hpp"
#include "vec3f.hpp"


using namespace imu_data_acquisition;

int number_of_samples = 3000;
vec3f acceleration, angular_velocity, magnetic_field, earth_acceleration,
    euler_orientation, pni_acceleration;
double initial_time = 0, delta_time = 0;
double target_latitude = 15.35, target_longitude = 73.68, target_distance,
       target_bearing;
extern SoftwareSerial pni_Serial;
extern SoftwareSerial gps_serial_;


extern Imu imu;

#endif // MAIN_HPP
