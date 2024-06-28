#include "imu.hpp"

namespace gps_with_imu {
Imu::Imu() {}
Imu::~Imu() {}

void Imu::initialize() {
  accelerometer_status_ = accelerometer_.begin();
  if (accelerometer_status_)
    Serial.println("Accelerometer initialized successfully!");
  else
    Serial.println("Failed to initialize the accelerometer!");
  magnetometer_status_ = magnetometer_.begin();
  if (magnetometer_status_)
    Serial.println("magnetometer initialized successfully!");
  else
    Serial.println("Failed to initialize the magnetometer!");
  gyroscope_status_ = 0;
  if (gyroscope_status_)
    Serial.println("Gyroscope initialized successfully!");
  else
    Serial.println("Failed to initialize the gyroscope!");
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
  // Serial.print("ax: ");
  // Serial.print(raw_accelerometer_reading_.x, 6);
  // Serial.print("   ");
  // Serial.print("ay: ");
  // Serial.print(raw_accelerometer_reading_.y, 6);
  // Serial.print("   ");
  // Serial.print("az: ");
  // Serial.print(raw_accelerometer_reading_.z, 6);
  // Serial.print("   ");
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

void Imu::MPUinitialize() {
  mpu9250_status_ = mpu9250_.init();
  if (mpu9250_status_)
    Serial.println("MPU9250 initialized successfully!");
  else
    Serial.println("Failed to initialize MPU9250!");
  mpu9250_mag_status_ = mpu9250_.initMagnetometer();
  if (mpu9250_status_)
    Serial.println("MPU9250 magnetometer initialized successfully!");
  else
    Serial.println("Failed to initialize MPU9250 magnteometer!");
  mpu9250_.enableGyrDLPF();
  mpu9250_.setGyrDLPF(MPU9250_DLPF_6);           // lowest noise
  mpu9250_.setGyrRange(MPU9250_GYRO_RANGE_250);  // highest resolution
  mpu9250_.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu9250_.enableAccDLPF(true);
  mpu9250_.setAccDLPF(MPU9250_DLPF_6);
  mpu9250_.setSampleRateDivider(5);
  mpu9250_.setMagOpMode(AK8963_CONT_MODE_100HZ);
}

void Imu::MPUcalculateOffsets(int number_of_samples) {
  int i;
  for (i = 0; i <= number_of_samples; i++) {
    xyzFloat accelg = mpu9250_.getGValues();
    xyzFloat gyro = mpu9250_.getGyrValues();
    raw_accelerometer_reading_ = {float(accelg.x * G), float(accelg.y * G),
                                  float(accelg.z * G)};
    raw_gyroscope_reading_ = {gyro.x, gyro.y, gyro.z};

    accelerometer_offset_.axis.x += raw_accelerometer_reading_.x;
    accelerometer_offset_.axis.y += raw_accelerometer_reading_.y;
    accelerometer_offset_.axis.z += raw_accelerometer_reading_.z;
    gyroscope_offset_.axis.x += raw_gyroscope_reading_.x;
    gyroscope_offset_.axis.y += raw_gyroscope_reading_.y;
    gyroscope_offset_.axis.z += raw_gyroscope_reading_.z;
  }
  accelerometer_offset_.axis.x /= number_of_samples;
  accelerometer_offset_.axis.y /= number_of_samples;
  accelerometer_offset_.axis.z /= number_of_samples;
  gyroscope_offset_.axis.x /= number_of_samples;
  gyroscope_offset_.axis.y /= number_of_samples;
  gyroscope_offset_.axis.z /= number_of_samples;
  accelerometer_offset_.axis.z -= G;
}

void Imu::MPUupdateReadings() {
  xyzFloat accelg = mpu9250_.getGValues();
  xyzFloat gyro = mpu9250_.getGyrValues();
  xyzFloat mag = mpu9250_.getMagValues();
  raw_accelerometer_reading_ = {float(accelg.x * G), float(accelg.y * G),
                                float(accelg.z * G)};
  raw_gyroscope_reading_ = {gyro.x, gyro.y, gyro.z};
  raw_magnetometer_reading_ = {mag.x, mag.y, mag.z};
}

void Imu::calibrateAccelerometer() {
  FusionVector acceleration = {raw_accelerometer_reading_.x,
                               raw_accelerometer_reading_.y,
                               raw_accelerometer_reading_.z};
  acceleration = FusionCalibrationInertial(
      acceleration, accelerometer_misalignment_, accelerometer_sensitivity_,
      accelerometer_offset_);
  acceleration_ = {acceleration.axis.x, acceleration.axis.y,
                   acceleration.axis.z};
}
void Imu::calibrateGyroscope() {
  FusionVector angular_velocity = {raw_gyroscope_reading_.x,
                                   raw_gyroscope_reading_.y,
                                   raw_gyroscope_reading_.z};
  angular_velocity =
      FusionCalibrationInertial(angular_velocity, gyroscope_misalignment_,
                                gyroscope_sensitivity_, gyroscope_offset_);
  angular_velocity_ = {angular_velocity.axis.x, angular_velocity.axis.y,
                       angular_velocity.axis.z};
}

void Imu::calibrateMagnetometer() {
  FusionVector magnetic_field = {raw_magnetometer_reading_.x,
                                 raw_magnetometer_reading_.y,
                                 raw_magnetometer_reading_.z};
  magnetic_field = FusionCalibrationMagnetic(magnetic_field, soft_iron_matrix_,
                                             hard_iron_offset_);
  magnetic_field_ = {magnetic_field.axis.x, magnetic_field.axis.y,
                     magnetic_field.axis.z};
}
void Imu::calibrateImu() {
  calibrateAccelerometer();
  calibrateGyroscope();
  calibrateMagnetometer();
}

void Imu::initializeAhrs() {
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = 0.5f,
      .gyroscopeRange = 250.0f,
      .accelerationRejection = 20.0f,
      .magneticRejection = 20.0f,
      .recoveryTriggerPeriod = 3 * SAMPLE_RATE,
  };
  FusionAhrsSetSettings(&ahrs, &settings);
  Serial.println("Ahrs initialized");
}
void Imu::updateOrientation() {
  FusionVector accelerometer = {acceleration_.x, acceleration_.y,
                                acceleration_.z};
  FusionVector gyroscope = {angular_velocity_.x, angular_velocity_.y,
                            angular_velocity_.z};
  FusionVector magnetometer = {magnetic_field_.x, magnetic_field_.y,
                               magnetic_field_.z};
  current_time = millis();
  deltaTime = current_time - prev_time;
  prev_time = current_time;
  if (start) {
    start = !start;
    deltaTime = 0;
  }
  deltaTime = deltaTime / 1000;

  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

  FusionVector translation = FusionAhrsGetEarthAcceleration(&ahrs);
  FusionEuler rotation =
      FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  earth_acceleration_ = {translation.axis.x, translation.axis.y,
                         translation.axis.z};
  euler_orientation_ = {rotation.angle.roll, rotation.angle.pitch,
                        rotation.angle.yaw};
  if (euler_orientation_.z < 0) {
    euler_orientation_.z = 360 + euler_orientation_.z;
  }
  // Serial.println("Earth acceleration obtained");
}

vec3f Imu::getAcceleration() { return acceleration_; }
vec3f Imu::getAngularVelocity() { return angular_velocity_; }
vec3f Imu::getMagneticField() { return magnetic_field_; }
vec3f Imu::getEarthAcceleration() { return earth_acceleration_; }
vec3f Imu::getEulerOrientation() { return euler_orientation_; }

}  // namespace gps_with_imu