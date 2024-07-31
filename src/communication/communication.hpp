#ifndef IMU_DATA_ACQUISITION_COMMUNICATION_COMMUNICATION_HPP
#define IMU_DATA_ACQUISITION_COMMUNICATION_COMMUNICATION_HPP

#include <geometry_msgs/Vector3.h>
#include <ros.h>

#include "vec3f.hpp"

namespace imu_data_acquisition
{
    void initializeCommunication();
    void sendData(vec3f acceleration, vec3f earth_acceleration, vec3f angular_velocity, vec3f magnetic_field,
                  vec3f orientation);
    void checkForCommands();
} // namespace imu_data_acquisition
#endif // IMU_DATA_ACQUISITON_COMMUNICATION_COMMUNICATION_HPP&