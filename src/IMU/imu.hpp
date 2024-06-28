#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <MPU9250_WE.h>
#include <Wire.h>

#include "Fusion.h"
#include "vec3f.hpp"
#define MPU9250_ADDR 0x68
#define G 9.80665
#define SAMPLE_RATE 100
#define ACCELERO_METER_SENSITIVITY_2 0.00003051757812 * 2
#define ACCELERO_METER_SENSITIVITY_4 0.00006103515625 * 2
#define ACCELERO_METER_SENSITIVITY_8 0.0001220703125 * 2
#define ACCELERO_METER_SENSITIVITY_16 0.000244140625 * 2
#define GYROSCOPE_SENSITIVITY_250 0.003814697266 * 2
#define GYROSCOPE_SENSITIVITY_500 0.007629394531 * 2
#define GYROSCOPE_SENSITIVITY_1000 0.01525878906 * 2
#define GYROSCOPE_SENSITIVITY_2000 0.03051757812 * 2

namespace gps_with_imu {
class Imu {
 private:
  FusionOffset offset;
  FusionAhrs ahrs;
  MPU9250_WE mpu9250_ = MPU9250_WE(MPU9250_ADDR);
  Adafruit_LSM303_Accel_Unified accelerometer_ =
      Adafruit_LSM303_Accel_Unified(54321);
  Adafruit_LSM303_Mag_Unified magnetometer_ =
      Adafruit_LSM303_Mag_Unified(12345);
  sensors_event_t accelerometer_event, magnetometer_event;
  const FusionMatrix gyroscope_misalignment_ = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                                0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector gyroscope_sensitivity_ = {1.0f, 1.0f, 1.0f};
  FusionVector gyroscope_offset_ = {0.0f, 0.0f, 0.0f};

  const FusionMatrix accelerometer_misalignment_ = {
      1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector accelerometer_sensitivity_ = {1.0f, 1.0f, 1.0f};
  FusionVector accelerometer_offset_ = {0.0868, 0.31819, -1.0666};

  FusionMatrix soft_iron_matrix_ = {0.971,  -0.048, -0.024, -0.048, 0.972,
                                    -0.011, -0.024, -0.011, 1.063};
  FusionVector hard_iron_offset_ = {57.59, 88.22, 74.47};

  vec3f raw_accelerometer_reading_, raw_gyroscope_reading_,
      raw_magnetometer_reading_;
  vec3f acceleration_, angular_velocity_, magnetic_field_;
  vec3f earth_acceleration_, euler_orientation_;
  bool mpu9250_status_, mpu9250_mag_status_;
  ;
  bool accelerometer_status_, magnetometer_status_, gyroscope_status_;
  bool start = true;
  double current_time = millis(), prev_time = current_time, deltaTime = 0;
  void calibrateAccelerometer();
  void calibrateGyroscope();
  void calibrateMagnetometer();

 public:
  Imu(/* args */);
  ~Imu();
  void initialize();
  void calculateOffsets(int number_of_samples);
  void UpdateReadings();
  void calibrateImu();
  void initializeAhrs();
  void updateOrientation();
  void MPUinitialize();
  void MPUcalculateOffsets(int number_of_samples);
  void MPUupdateReadings();
  vec3f getAcceleration();
  vec3f getAngularVelocity();
  vec3f getMagneticField();
  vec3f getEarthAcceleration();
  vec3f getEulerOrientation();
};
}  // namespace gps_with_imu