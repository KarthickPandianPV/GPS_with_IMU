#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "Fusion.h"
#include "vec3f.hpp"
#define G 9.80665
#define SAMPLE_RATE 100

namespace gps_with_imu {
class Imu {
 private:
  FusionOffset offset;
  FusionAhrs ahrs;
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
  FusionVector acceleration_, angular_velocity_, magnetic_field_;

  bool accelerometer_status_, magnetometer_status_, gyroscope_status_;
  bool start = true;
  double current_time = millis(), prev_time = current_time, deltaTime = 0;

 public:
  Imu(/* args */);
  ~Imu();
  void initialize();
  void calculateOffsets(int number_of_samples);
  void UpdateReadings();
  vec3f calibrateAccelerometer();
  vec3f calibrateMagnetometer();
  void initializeAhrs();
  vec3f getGlobalFrameAcceleration();
};
}  // namespace gps_with_imu