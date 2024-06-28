#include "communication.hpp"

ros::NodeHandle nh;

geometry_msgs::Vector3 acceleration_msg;
geometry_msgs::Vector3 magnetic_field_msg;
geometry_msgs::Vector3 angular_velocity_msg;
geometry_msgs::Vector3 earth_acceleration_msg;
geometry_msgs::Vector3 orientation_msg;

ros::Publisher magnetic_field_publisher("magnetic_field", &magnetic_field_msg);
ros::Publisher acceleration_publisher("acceleration", &acceleration_msg);
ros::Publisher angular_velocity_publisher("angular_velocity",
                                          &angular_velocity_msg);
ros::Publisher earth_acceleration_publisher("earth_acceleration",
                                            &earth_acceleration_msg);
ros::Publisher orientation_publisher("euler_orientation", &orientation_msg);
namespace gps_with_imu {
void initializeCommunication() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(acceleration_publisher);
  nh.advertise(magnetic_field_publisher);
  nh.advertise(angular_velocity_publisher);
  nh.advertise(earth_acceleration_publisher);
  nh.advertise(orientation_publisher);
  Serial.println("ROS communication initialized");
}
void sendData(vec3f acceleration, vec3f angular_velocity, vec3f magnetic_field,
              vec3f earth_acceleration, vec3f euler_orientation) {
  acceleration_msg.x = acceleration.x;
  acceleration_msg.y = acceleration.y;
  acceleration_msg.z = acceleration.z;

  magnetic_field_msg.x = magnetic_field.x;
  magnetic_field_msg.y = magnetic_field.y;
  magnetic_field_msg.z = magnetic_field.z;

  angular_velocity_msg.x = angular_velocity.x;
  angular_velocity_msg.y = angular_velocity.y;
  angular_velocity_msg.z = angular_velocity.z;

  earth_acceleration_msg.x = earth_acceleration.x;
  earth_acceleration_msg.y = earth_acceleration.y;
  earth_acceleration_msg.z = earth_acceleration.z;

  orientation_msg.x = euler_orientation.x;
  orientation_msg.y = euler_orientation.y;
  orientation_msg.z = euler_orientation.z;

  acceleration_publisher.publish(&acceleration_msg);
  magnetic_field_publisher.publish(&magnetic_field_msg);
  angular_velocity_publisher.publish(&angular_velocity_msg);
  earth_acceleration_publisher.publish(&earth_acceleration_msg);
  orientation_publisher.publish(&orientation_msg);
  // Serial.println("Data sent");
}

void checkForCommands() { nh.spinOnce(); }
}  // namespace gps_with_imu