#include "imu.hpp"

namespace imu_data_acquisition
{
  Imu::Imu() {}
  Imu::~Imu() {}
  void Imu::pniInitialize(SoftwareSerial &pni_Serial)
  {
    pni_Serial.begin(38400);
    pni_Serial.write(setDataComponents, sizeof(setDataComponents));
    Serial.println("Pni initialized successfully");
  }

  void Imu::pniUpdateReadings(SoftwareSerial &pni_Serial)
  {
    pni_Serial.write(getData, sizeof(getData));
    while (pni_Serial.available() > 0)
    {
      receivedData[byteCount] = pni_Serial.read();
      byteCount++;
    }
    // for (int k = 0; k < byteCount; k++)  \\ TO print received bytes
    // {
    //   Serial.print(receivedData[k], HEX);
    //   Serial.print(" ");
    //   i = 1;
    // }
    // if (i == 1)
    //   Serial.println();
    // i = 0;
    byteCount = 0;
    for (int k = 0; k < 4; k++)
    {
      kmx[k] = receivedData[k + 5];
      kmy[k] = receivedData[k + 10];
      kmz[k] = receivedData[k + 15];
      kax[k] = receivedData[k + 20];
      kay[k] = receivedData[k + 25];
      kaz[k] = receivedData[k + 30];
    }
    raw_magnetometer_reading_.x = hexToDouble(kmx);
    raw_magnetometer_reading_.y = hexToDouble(kmy);
    raw_magnetometer_reading_.z = hexToDouble(kmz);
    pni_raw_acceleration.x = (hexToDouble(kax));
    pni_raw_acceleration.y = (hexToDouble(kay));
    pni_raw_acceleration.z = (hexToDouble(kaz));
  }
  void Imu::pniCalculateOffset(SoftwareSerial &pni_Serial, int number_of_samples)
  {
    int j = 0;
    for (int i = 0; i < number_of_samples; i++)
    {
      pni_Serial.write(getData, sizeof(getData));
      while (pni_Serial.available() > 0)
      {
        receivedData[byteCount] = pni_Serial.read();
        byteCount++;
      }
      byteCount = 0;
      delay(35);
      for (int k = 0; k < 4; k++)
      {
        kax[k] = receivedData[k + 20];
        kay[k] = receivedData[k + 25];
        kaz[k] = receivedData[k + 30];
      }
      pni_raw_acceleration.x = (hexToDouble(kax));
      pni_raw_acceleration.y = (hexToDouble(kay));
      pni_raw_acceleration.z = (hexToDouble(kaz));

      pni_accelerometer_offset_.axis.x += pni_raw_acceleration.x;
      pni_accelerometer_offset_.axis.y += pni_raw_acceleration.y;
      pni_accelerometer_offset_.axis.z += pni_raw_acceleration.z;
    }
    pni_accelerometer_offset_.axis.x /= number_of_samples;
    pni_accelerometer_offset_.axis.y /= number_of_samples;
    pni_accelerometer_offset_.axis.z /= number_of_samples;
    pni_accelerometer_offset_.axis.z -= 1;
  }

  void Imu::LSMinitialize()
  {
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
  }

  void Imu::LSMcalculateOffsets(int number_of_samples)
  {
    int i;
    long initial = millis(), delta;
    for (i = 0; i <= number_of_samples; i++)
    {
      accelerometer_.getEvent(&accelerometer_event);
      magnetometer_.getEvent(&magnetometer_event);
      raw_accelerometer_reading_ = {accelerometer_event.acceleration.x,
                                    accelerometer_event.acceleration.y,
                                    accelerometer_event.acceleration.z};
      raw_magnetometer_reading_ = {magnetometer_event.magnetic.x,
                                   magnetometer_event.magnetic.y,
                                   magnetometer_event.magnetic.z};
      accelerometer_offset_.axis.x += raw_accelerometer_reading_.x;
      accelerometer_offset_.axis.y += raw_accelerometer_reading_.y;
      accelerometer_offset_.axis.z += raw_accelerometer_reading_.z;
    }
    delta = millis() - initial;
    accelerometer_offset_.axis.x /= number_of_samples;
    accelerometer_offset_.axis.y /= number_of_samples;
    accelerometer_offset_.axis.z /= number_of_samples;
    accelerometer_offset_.axis.z -= G;
  }

  void Imu::LSMupdateReadings()
  {
    accelerometer_.getEvent(&accelerometer_event);
    magnetometer_.getEvent(&magnetometer_event);
    raw_accelerometer_reading_.x = accelerometer_event.acceleration.x;
    raw_accelerometer_reading_.y = accelerometer_event.acceleration.y;
    raw_accelerometer_reading_.z = accelerometer_event.acceleration.z;
    raw_magnetometer_reading_.x = magnetometer_event.magnetic.x;
    raw_magnetometer_reading_.y = magnetometer_event.magnetic.y;
    raw_magnetometer_reading_.z = magnetometer_event.magnetic.z;
  }

  void Imu::MPUinitialize()
  {
    mpu6050_status_ = mpu6050_.begin();
    if (!mpu6050_status_)
      Serial.println("Failed to initialize MPU9250!");
    else
      Serial.println("ax ay az eax eay eaz mx my mz roll pitch yaw");
    mpu6050_.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu6050_.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu6050_.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  void Imu::MPUcalculateOffsets(int number_of_samples)
  {
    // int i;
    for (int i = 0; i <= number_of_samples; i++)
    {
      sensors_event_t accelerometer, gyroscope, temp;
      mpu6050_.getEvent(&accelerometer, &gyroscope, &temp);
      raw_accelerometer_reading_ = {accelerometer.acceleration.y, accelerometer.acceleration.x,
                                    accelerometer.acceleration.z};
      raw_gyroscope_reading_ = {(gyroscope.gyro.y), gyroscope.gyro.x, (gyroscope.gyro.z)};

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

  void Imu::MPUupdateReadings()
  {
    sensors_event_t accelerometer, gyroscope, temp;
    mpu6050_.getEvent(&accelerometer, &gyroscope, &temp);

    raw_accelerometer_reading_ = {accelerometer.acceleration.y, accelerometer.acceleration.x,
                                  accelerometer.acceleration.z};
    raw_gyroscope_reading_ = {(gyroscope.gyro.y), gyroscope.gyro.x, (gyroscope.gyro.z)};
  }

  void Imu::calibrateAccelerometer()
  {
    FusionVector acceleration = {raw_accelerometer_reading_.x,
                                 raw_accelerometer_reading_.y,
                                 raw_accelerometer_reading_.z};
    FusionVector pni_acceleration = {pni_raw_acceleration.x, pni_raw_acceleration.y, pni_raw_acceleration.z};

    acceleration = FusionCalibrationInertial(
        acceleration, accelerometer_misalignment_, accelerometer_sensitivity_,
        accelerometer_offset_);
    pni_acceleration = FusionCalibrationInertial(pni_acceleration, accelerometer_misalignment_, accelerometer_sensitivity_, pni_accelerometer_offset_);

    acceleration_ = {acceleration.axis.x, acceleration.axis.y,
                     acceleration.axis.z};
    pni_acceleration.axis.x *= G;
    pni_acceleration.axis.y *= G;
    pni_acceleration.axis.z *= G;
    pni_acceleration_ = {pni_acceleration.axis.x, pni_acceleration.axis.y, pni_acceleration.axis.z};
  }
  void Imu::calibrateGyroscope()
  {
    FusionVector angular_velocity = {raw_gyroscope_reading_.x,
                                     raw_gyroscope_reading_.y,
                                     raw_gyroscope_reading_.z};
    angular_velocity =
        FusionCalibrationInertial(angular_velocity, gyroscope_misalignment_,
                                  gyroscope_sensitivity_, gyroscope_offset_);
    angular_velocity = FusionOffsetUpdate(&offset, angular_velocity);
    angular_velocity_ = {angular_velocity.axis.x, angular_velocity.axis.y,
                         angular_velocity.axis.z};
  }

  void Imu::calibrateMagnetometer()
  {
    FusionVector magnetic_field = {raw_magnetometer_reading_.x,
                                   raw_magnetometer_reading_.y,
                                   raw_magnetometer_reading_.z};
    magnetic_field = FusionCalibrationMagnetic(magnetic_field, soft_iron_matrix_,
                                               hard_iron_offset_);
    magnetic_field_ = {magnetic_field.axis.x, magnetic_field.axis.y,
                       magnetic_field.axis.z};
  }
  void Imu::calibrateImu()
  {
    calibrateAccelerometer();
    calibrateGyroscope();
    calibrateMagnetometer();
  }

  void Imu::initializeAhrs()
  {
    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 2,
        .gyroscopeRange = 250.0f,
        .accelerationRejection = 15.0f,
        .magneticRejection = 15.0f,
        .recoveryTriggerPeriod = 5,
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    Serial.println("Ahrs initialized");
  }
  void Imu::updateOrientation()
  {
    FusionVector accelerometer = {acceleration_.x, acceleration_.y,
                                  acceleration_.z};
    FusionVector gyroscope = {angular_velocity_.x, angular_velocity_.y,
                              angular_velocity_.z};
    FusionVector magnetometer = {magnetic_field_.x, magnetic_field_.y,
                                 magnetic_field_.z};
    current_time = millis();
    deltaTime = current_time - prev_time;
    prev_time = current_time;
    if (start)
    {
      start = !start;
      deltaTime = 0;
    }
    deltaTime = deltaTime / 1000;

    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
    FusionEuler rotation =
        FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    euler_orientation_ = {rotation.angle.roll, rotation.angle.pitch,
                          rotation.angle.yaw};

    if (euler_orientation_.z < 0)
    {
      euler_orientation_.z = 360 + euler_orientation_.z;
    }

    FusionVector translation = FusionAhrsGetEarthAcceleration(&ahrs);
    earth_acceleration_ = {translation.axis.x, translation.axis.y,
                           translation.axis.z};
  }

  vec3f Imu::getAcceleration() { return acceleration_; }
  vec3f Imu::getAngularVelocity() { return angular_velocity_; }
  vec3f Imu::getMagneticField() { return magnetic_field_; }
  vec3f Imu::getEarthAcceleration() { return earth_acceleration_; }
  vec3f Imu::getEulerOrientation() { return euler_orientation_; }
  vec3f Imu::getPniAcceleration() { return pni_acceleration_; }
  float Imu::hexToDouble(byte *hex)
  {
    byte sign, exponent, mantissa[3];
    float value = 0;
    sign = (hex[0] & 0x80) >> 7;
    exponent = ((hex[0] & 0x7F) << 1) | ((hex[1] & 0x80) >> 7);
    mantissa[0] = ((hex[1] & 0x7F) << 1) | ((hex[2] & 0x80) >> 7);
    mantissa[1] = ((hex[2] & 0x7F) << 1) | ((hex[3] & 0x80) >> 7);
    mantissa[2] = ((hex[3] & 0x7F) << 1) & 0xFE;
    value = pow(-1, sign) * pow(2, exponent - 127) * (1 + (mantissa[0] * pow(2, -8)) + (mantissa[1] * pow(2, -16)) + (mantissa[2] * pow(2, -24)));
    return value;
  }
} // namespace IMU_DATA_ACQUISITION