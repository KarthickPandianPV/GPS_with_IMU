#ifndef GPS_WITH_IMU_COMMUNICATION_COMMUNICATION_HPP
#define GPS_WITH_IMU_COMMUNICATION_COMMUNICATION_HPP

#include <geometry_msgs/Vector3.h>
#include <ros.h>

#include "vec3f.hpp"

// extern ros::NodeHandle nh;

// extern geometry_msgs::Vector3 acceleration_msg;
// extern geometry_msgs::Vector3 magnetic_field_msg;
// extern geometry_msgs::Vector3 velocity_msg;
// extern geometry_msgs::Vector3 distance_msg;

// extern ros::Publisher magnetic_field_publisher;
// extern ros::Publisher acceleration_publisher;
// extern ros::Publisher velocity_publisher;
// extern ros::Publisher distance_publisher;
namespace gps_with_imu {
void initializeCommunication();
void sendData(vec3f acceleration, vec3f angular_velocity, vec3f magnetic_field,
              vec3f earth_acceleration, vec3f orientation);
void checkForCommands();
}  // namespace gps_with_imu
#endif  // GPS_WITH_IMU_COMMUNICATION_COMMUNICATION_HPP&