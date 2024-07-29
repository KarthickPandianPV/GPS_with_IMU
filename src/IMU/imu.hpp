#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <MPU9250_WE.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// #include <Arduino.h>

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

namespace gps_with_imu
{
    class Imu
    {
    private:
        int byteCount = 0, i = 0;
        byte setDataComponents[12] = {0x00, 0x0C, 0x03, 0x06, 0x1b, 0x1c, 0x1d, 0x15, 0x16, 0x17, 0xcb, 0xa6};
        byte getData[5] = {0x00, 0x05, 0x04, 0xbf, 0x71};

        FusionOffset offset;
        FusionAhrs ahrs;
        FusionEuler euler;
        Adafruit_MPU6050 mpu6050_;
        // MPU9250_WE mpu9250_ = MPU9250_WE(MPU9250_ADDR);
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
        FusionVector accelerometer_offset_ = {-0.068402, -0.0292, 1.003778};
        FusionVector pni_accelerometer_offset_ = {0, 0, 0};
        FusionMatrix soft_iron_matrix_ = {0.969, 0.001, 0.008, -0.001, 1.012, -0.011, 0.008, -0.011, 1.020};
        FusionVector hard_iron_offset_ = {1.19, -0.34, -0.13};
        vec3f pni_raw_acceleration;
        vec3f raw_accelerometer_reading_, raw_gyroscope_reading_,
            raw_magnetometer_reading_;
        vec3f acceleration_, angular_velocity_, magnetic_field_, pni_acceleration_;
        vec3f earth_acceleration_, euler_orientation_;
        bool mpu6050_status_, mpu9250_mag_status_;
        ;
        bool accelerometer_status_, magnetometer_status_, gyroscope_status_;
        bool start = true;
        double current_time = millis(), prev_time = current_time, deltaTime = 0;
        void calibrateAccelerometer();
        void calibrateGyroscope();
        void calibrateMagnetometer();
        float hexToDouble(byte *hex);

    public:
        Imu(/* args */);
        ~Imu();
        byte receivedData[36], kmx[4], kmy[4], kmz[4], kax[4], kay[4], kaz[4];
        void initialize();
        void calculateOffsets(int number_of_samples);
        void UpdateReadings();
        void calibrateImu();
        void initializeAhrs();
        void updateOrientation();
        void MPUinitialize();
        void MPUcalculateOffsets(int number_of_samples);
        void MPUupdateReadings();
        void pniInitialize(SoftwareSerial &pni_Serial);
        void pniUpdateReadings(SoftwareSerial &pni_Serial);
        void pniCalculateOffset(SoftwareSerial &pni_Serial, int number_of_samples);
        vec3f getAcceleration();
        vec3f getAngularVelocity();
        vec3f getMagneticField();
        vec3f getEarthAcceleration();
        vec3f getEulerOrientation();
        vec3f getPniAcceleration();
    };
} // namespace gps_with_imu