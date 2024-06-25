#include "imu.hpp"

namespace gps_with_imu {
Imu::Imu() {}
Imu::~Imu() {}

void Imu::initialize() {
  accelerometer_status_ = accelerometer_.begin();
  // if (accelerometer_status_)
  //   Serial.println("Accelerometer initialized successfully!");
  // else
  //   Serial.println("Failed to initialize the accelerometer!");
  magnetometer_status_ = magnetometer_.begin();
  // if (magnetometer_status_)
  //   Serial.println("magnetometer initialized successfully!");
  // else
  //   Serial.println("Failed to initialize the magnetometer!");
  gyroscope_status_ = 0;
  // if (gyroscope_status_)
  //   Serial.println("Gyroscope initialized successfully!");
  // else
  //   Serial.println("Failed to initialize the gyroscope!");
}

void Imu::calculateOffsets(int number_of_samples) {
  int i;
  long initial = millis(), delta;
  for (i = 0; i <= number_of_samples; i++) {
    accelerometer_.getEvent(&accelerometer_event);
    magnetometer_.getEvent(&magnetometer_event);
    raw_accelerometer_reading_ = {accelerometer_event.acceleration.x,
                                  accelerometer_event.acceleration.y,
                                  accelerometer_event.acceleration.z};
    raw_magnetometer_reading_ = {magnetometer_event.magnetic.x,
                                 magnetometer_event.magnetic.y,
                                 magnetometer_event.magnetic.z};
    raw_gyroscope_reading_ = {0.0f, 0.0f, 0.0f};

    accelerometer_offset_.axis.x += raw_accelerometer_reading_.x;
    accelerometer_offset_.axis.y += raw_accelerometer_reading_.y;
    accelerometer_offset_.axis.z += raw_accelerometer_reading_.z;
    gyroscope_offset_.axis.x += raw_gyroscope_reading_.x;
    gyroscope_offset_.axis.y += raw_gyroscope_reading_.y;
    gyroscope_offset_.axis.z += raw_gyroscope_reading_.z;
  }
  delta = millis() - initial;
  accelerometer_offset_.axis.x /= number_of_samples;
  accelerometer_offset_.axis.y /= number_of_samples;
  accelerometer_offset_.axis.z /= number_of_samples;
  gyroscope_offset_.axis.x /= number_of_samples;
  gyroscope_offset_.axis.y /= number_of_samples;
  gyroscope_offset_.axis.z /= number_of_samples;
  accelerometer_offset_.axis.z -= G;
  // Serial.print("Offset calculation time: ");
  // Serial.println(delta);
  // Serial.print("ax: ");
  // Serial.print(accelerometer_offset_.axis.x, 6);
  // Serial.print("   ");
  // Serial.print("ay: ");
  // Serial.print(accelerometer_offset_.axis.y, 6);
  // Serial.print("   ");
  // Serial.print("az: ");
  // Serial.print(accelerometer_offset_.axis.z, 6);
  // Serial.println("   ");
}

void Imu::UpdateReadings() {
  accelerometer_.getEvent(&accelerometer_event);
  magnetometer_.getEvent(&magnetometer_event);
  raw_accelerometer_reading_.x = accelerometer_event.acceleration.x;
  raw_accelerometer_reading_.y = accelerometer_event.acceleration.y;
  raw_accelerometer_reading_.z = accelerometer_event.acceleration.z;
  raw_magnetometer_reading_.x = magnetometer_event.magnetic.x;
  raw_magnetometer_reading_.y = magnetometer_event.magnetic.y;
  raw_magnetometer_reading_.z = magnetometer_event.magnetic.z;
  raw_gyroscope_reading_ = {0, 0, 0};
  Serial.print("ax: ");
  Serial.print(raw_accelerometer_reading_.x, 6);
  Serial.print("   ");
  Serial.print("ay: ");
  Serial.print(raw_accelerometer_reading_.y, 6);
  Serial.print("   ");
  Serial.print("az: ");
  Serial.print(raw_accelerometer_reading_.z, 6);
  Serial.print("   ");
  // Serial.print("Offset calculation time: ");
  // Serial.println(delta);
  // Serial.print("ax: ");
  // Serial.print(accelerometer_offset_.axis.x, 6);
  // Serial.print("   ");
  // Serial.print("ay: ");
  // Serial.print(accelerometer_offset_.axis.y, 6);
  // Serial.print("   ");
  // Serial.print("az: ");
  // Serial.print(accelerometer_offset_.axis.z, 6);
  // Serial.println("   ");
}

vec3f Imu::calibrateAccelerometer() {
  acceleration_.axis.x =
      raw_accelerometer_reading_.x - accelerometer_offset_.axis.x;
  acceleration_.axis.y =
      raw_accelerometer_reading_.y - accelerometer_offset_.axis.y;
  acceleration_.axis.z =
      raw_accelerometer_reading_.z - accelerometer_offset_.axis.z;
  vec3f acceleration = {acceleration_.axis.x, acceleration_.axis.y,
                        acceleration_.axis.z};
  return acceleration;
}

vec3f Imu::calibrateMagnetometer() {
  magnetic_field_ = {raw_magnetometer_reading_.x, raw_magnetometer_reading_.y,
                     raw_magnetometer_reading_.z};
  magnetic_field_ = FusionCalibrationMagnetic(
      magnetic_field_, soft_iron_matrix_, hard_iron_offset_);
  vec3f magnetic_field = {magnetic_field_.axis.x, magnetic_field_.axis.y,
                          magnetic_field_.axis.z};
  return magnetic_field;
}

void Imu::initializeAhrs() {
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 1.0f,
      .gyroscopeRange = 250.0f,
      .accelerationRejection = 20.0f,
      .magneticRejection = 20.0f,
      .recoveryTriggerPeriod = 3 * SAMPLE_RATE,
  };
  FusionAhrsSetSettings(&ahrs, &settings);
  Serial.println("Ahrs initialized");
}
vec3f Imu::getGlobalFrameAcceleration() {
  current_time = millis();
  deltaTime = current_time - prev_time;
  prev_time = current_time;
  if (start) {
    start = !start;
    deltaTime = 0;
  }
  deltaTime = deltaTime / 1000;

  FusionAhrsUpdate(&ahrs, angular_velocity_, acceleration_, magnetic_field_,
                   deltaTime);

  acceleration_ = FusionAhrsGetEarthAcceleration(&ahrs);

  vec3f earth_acceleration = {acceleration_.axis.x, acceleration_.axis.y,
                              acceleration_.axis.z};
  // Serial.println("Earth acceleration obtained");
  return earth_acceleration;
}
}  // namespace gps_with_imu