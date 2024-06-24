#include "communication.hpp"

ros::NodeHandle nh;

geometry_msgs::Vector3 acceleration_msg;
geometry_msgs::Vector3 magnetic_field_msg;
geometry_msgs::Vector3 velocity_msg;
geometry_msgs::Vector3 distance_msg;

ros::Publisher magnetic_field_publisher("magnetic_field", &magnetic_field_msg);
ros::Publisher acceleration_publisher("acceleration", &acceleration_msg);
ros::Publisher velocity_publisher("velocity", &velocity_msg);
ros::Publisher distance_publisher("distance", &distance_msg);

void initializeCommunication() {
  nh.initNode();
  nh.advertise(acceleration_publisher);
  nh.advertise(magnetic_field_publisher);
  nh.advertise(velocity_publisher);
  nh.advertise(distance_publisher);
  //   Serial.println("ROS communication initialized");
}
void sendData(vec3f& magnetic_field, vec3f& acceleration, vec3f& velocity,
              vec3f& distance) {
  acceleration_msg.x = acceleration.x;
  acceleration_msg.y = acceleration.y;
  acceleration_msg.z = acceleration.z;

  magnetic_field_msg.x = magnetic_field.x;
  magnetic_field_msg.y = magnetic_field.y;
  magnetic_field_msg.z = magnetic_field.z;

  velocity_msg.x = velocity.x;
  velocity_msg.y = velocity.y;
  velocity_msg.z = velocity.z;

  distance_msg.x = distance.x;
  distance_msg.y = distance.y;
  distance_msg.z = distance.z;
  acceleration_publisher.publish(&acceleration_msg);
  magnetic_field_publisher.publish(&magnetic_field_msg);
  velocity_publisher.publish(&velocity_msg);
  distance_publisher.publish(&distance_msg);
  //   Serial.println("Data sent");
}

void checkForCommands() { nh.spinOnce(); }