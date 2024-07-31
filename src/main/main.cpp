#include "main.hpp"

SoftwareSerial pni_Serial(PB8, PB9);
Imu imu;

void setup()
{
  Serial.begin(38400);
  while (!Serial)
  {
    Serial.println("Serial not available.");
    delay(1);
  }
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  initializeCommunication();
  imu.pniInitialize(pni_Serial);
  imu.MPUinitialize();
  imu.MPUcalculateOffsets(number_of_samples);
  imu.initializeAhrs();
}

void loop()
{
  imu.MPUupdateReadings();
  imu.pniUpdateReadings(pni_Serial);
  imu.calibrateImu();
  imu.updateOrientation();
  acceleration = imu.getAcceleration();
  angular_velocity = imu.getAngularVelocity();
  magnetic_field = imu.getMagneticField();
  earth_acceleration = imu.getEarthAcceleration();
  euler_orientation = imu.getEulerOrientation();
  pni_acceleration = imu.getPniAcceleration();
  sendData(acceleration, earth_acceleration, angular_velocity, magnetic_field, euler_orientation);
  checkForCommands();
  Serial.print(" ax: ");
  Serial.print(acceleration.x, 4);
  Serial.print(" ay: ");
  Serial.print(acceleration.y, 4);
  Serial.print(" az: ");
  Serial.print(acceleration.z, 4);

  Serial.print(" ax': ");
  Serial.print(earth_acceleration.x, 4);
  Serial.print(" ay': ");
  Serial.print(earth_acceleration.y, 4);
  Serial.print(" az': ");
  Serial.print(earth_acceleration.z, 4);

  Serial.print(" gx: ");
  Serial.print(angular_velocity.x, 4);
  Serial.print(" gy: ");
  Serial.print(angular_velocity.y, 5);
  Serial.print(" gz: ");
  Serial.print(angular_velocity.z, 5);

  Serial.print(" mx: ");
  Serial.print(magnetic_field.x, 4);
  Serial.print(" my: ");
  Serial.print(magnetic_field.y, 5);
  Serial.print(" mz: ");
  Serial.print(magnetic_field.z, 5);

  Serial.print(" roll: ");
  Serial.print(euler_orientation.x);
  Serial.print(" pitch: ");
  Serial.print(euler_orientation.y);
  Serial.print(" yaw: ");
  Serial.println(euler_orientation.z);
  delay(30);
}
