// #include "main.hpp"
// // byte setDataComponents[12] = {0x00, 0x0C, 0x03, 0x06, 0x1b, 0x1c, 0x1d, 0x15, 0x16, 0x17, 0xcb, 0xa6};
// // byte getData[5] = {0x00, 0x05, 0x04, 0xbf, 0x71};
// // int byteCount = 0;
// // byte receivedData[36];
// // int i = 0;
// // SoftwareSerial gps_serial_(GPS_RX_PIN, GPS_TX_PIN);
// long curr = 0, prev = millis();
// long delta = 0;
// void floatArrayToString(float *values, size_t count, char *buffer, size_t bufferSize)
// {
//   buffer[0] = '\0'; // Initialize buffer
//   for (size_t i = 0; i < count; i++)
//   {
//     char floatStr[20];                  // Buffer for individual float string
//     dtostrf(values[i], 1, 4, floatStr); // Convert float to string with 4 decimal places
//     strcat(buffer, floatStr);
//     if (i < count - 1)
//     {
//       strcat(buffer, ","); // Add a comma separator between values
//     }
//   }
// }
// SoftwareSerial pni_Serial(PB8, PB9);
// // Gps gps;
// Imu imu;
// void setup()
// {

//   Serial.begin(38400);
//   pni_Serial.begin(38400);
//   while (!Serial)
//   {
//     Serial.println("Serial not available.");
//     // delay(1);
//   }
//   // pni_Serial.write(setDataComponents, sizeof(setDataComponents));
//   Wire.setSCL(PB6);
//   Wire.setSDA(PB7);
//   Wire.begin();
//   // gps.initialize(gps_serial_);
//   // // gps.get_target_coordinates(target_latitude, target_longitude);
//   // Serial.print("Target latitude : ");
//   // Serial.println(target_latitude);
//   // Serial.print("Target longitude : ");
//   // Serial.println(target_longitude);
//   // delay(1000);
//   // initializeCommunication();
//   imu.pniInitialize(pni_Serial);
//   imu.MPUinitialize();
//   // imu.pniCalculateOffset(pni_Serial, number_of_samples);
//   imu.MPUcalculateOffsets(number_of_samples);
//   imu.initializeAhrs();
//   prev = millis();
// }

// void loop()
// {
//   // pni_Serial.write(getData, sizeof(getData));
//   // while (pni_Serial.available() > 0)
//   // {
//   //   imu.receivedData[byteCount] = pni_Serial.read();
//   //   byteCount++;
//   // }
//   // for (int k = 0; k < byteCount; k++)
//   // {
//   //   Serial.print(imu.receivedData[k], HEX);
//   //   Serial.print(" ");
//   //   i = 1;
//   // }
//   // if (i == 1)
//   //   Serial.println();
//   // i = 0;
//   // byteCount = 0;
//   // gps.get_gps_data(gps_serial_);
//   // if (gps_data_.gps_status) {
//   //   Serial.println("GPS is locked.");
//   //   gps.get_distance(target_distance, target_latitude, target_longitude);
//   //   Serial.print("Distance to target : ");
//   //   Serial.println(target_distance);
//   //   gps.get_direction(target_bearing, target_latitude, target_longitude);
//   //   Serial.print("Direction to target : ");
//   //   Serial.println(target_bearing);
//   // } else {
//   //   Serial.println("GPS is not locked..!");
//   // }
//   // // Serial.println(gps_data_.gprmc_message);
//   imu.MPUupdateReadings();
//   imu.pniUpdateReadings(pni_Serial);
//   imu.calibrateImu();
//   imu.updateOrientation();
//   acceleration = imu.getAcceleration();
//   angular_velocity = imu.getAngularVelocity();
//   magnetic_field = imu.getMagneticField();
//   earth_acceleration = imu.getEarthAcceleration();
//   euler_orientation = imu.getEulerOrientation();
//   pni_acceleration = imu.getPniAcceleration();
//   float values[15] = {acceleration.x, acceleration.y, acceleration.z, earth_acceleration.x, earth_acceleration.y, earth_acceleration.z, angular_velocity.x, angular_velocity.y, angular_velocity.z, magnetic_field.x, magnetic_field.y, magnetic_field.z, euler_orientation.x, euler_orientation.y, euler_orientation.z};
//   char buffer[500]; // Adjust size based on the expected string length

//   delta = millis() - prev;
//   prev = millis();
//   Serial.println(delta);
//   // floatArrayToString(values, 15, buffer, sizeof(buffer));

//   // sendData(acceleration, angular_velocity, magnetic_field, earth_acceleration,
//   //  euler_orientation, pni_acceleration);
//   // checkForCommands();
//   // Serial.print(" ax: ");
//   // Serial.print(acceleration.x, 4);
//   // Serial.print(" ay: ");
//   // Serial.print(acceleration.y, 4);
//   // Serial.print(" az: ");
//   // Serial.print(acceleration.z, 4);

//   // Serial.print(" ax': ");
//   // Serial.print(earth_acceleration.x, 4);
//   // Serial.print(" ay': ");
//   // Serial.print(earth_acceleration.y, 4);
//   // Serial.print(" az': ");
//   // Serial.print(earth_acceleration.z, 4);

//   // Serial.print(" gx: ");
//   // Serial.print(angular_velocity.x, 4);
//   // Serial.print(" gy: ");
//   // Serial.print(angular_velocity.y, 5);
//   // Serial.print(" gz: ");
//   // Serial.print(angular_velocity.z, 5);

//   // Serial.print(" mx: ");
//   // Serial.print(magnetic_field.x, 4);
//   // Serial.print(" my: ");
//   // Serial.print(magnetic_field.y, 5);
//   // Serial.print(" mz: ");
//   // Serial.print(magnetic_field.z, 5);

//   // Serial.print(" roll: ");
//   // Serial.print(euler_orientation.x);
//   // Serial.print(" pitch: ");
//   // Serial.print(euler_orientation.y);
//   // Serial.print(" yaw: ");
//   // Serial.println(euler_orientation.z);
//   // Serial.print("<");
//   // Serial.print(acceleration.x, sizeof(acceleration.x));
//   // Serial.print(" ");
//   // Serial.print(acceleration.y, sizeof(acceleration.y));
//   // Serial.print(" ");
//   // Serial.print(acceleration.z, sizeof(acceleration.z));
//   // Serial.print(" ");
//   // Serial.print(earth_acceleration.x, sizeof(earth_acceleration.x));
//   // Serial.print(" ");
//   // Serial.print(earth_acceleration.y, sizeof(earth_acceleration.y));
//   // Serial.print(" ");
//   // Serial.print(earth_acceleration.z, sizeof(earth_acceleration.z));
//   // Serial.print(" ");
//   // Serial.print(angular_velocity.x, 4);
//   // Serial.print(" ");
//   // Serial.print(angular_velocity.y, 4);
//   // Serial.print(" ");
//   // Serial.print(angular_velocity.z, 4);
//   // Serial.print(" ");
//   // Serial.print(magnetic_field.x, sizeof(magnetic_field.x));
//   // Serial.print(" ");
//   // Serial.print(magnetic_field.y, sizeof(magnetic_field.y));
//   // Serial.print(" ");
//   // Serial.print(magnetic_field.z, sizeof(magnetic_field.z));
//   // Serial.print(" ");
//   // Serial.print(euler_orientation.x, sizeof(euler_orientation.x));
//   // Serial.print(" ");
//   // Serial.print(euler_orientation.y, sizeof(euler_orientation.y));
//   // Serial.print(" ");
//   // Serial.println(euler_orientation.z, sizeof(euler_orientation.z));
//   // Serial.println(">");
//   // Serial.println(buffer); // Send the entire string
//   delay(30);
// }

// // //  if ((acceleration.x <= 0.1) && (acceleration.x >= -0.1)) {
// // //     acceleration.x = 0;
// // //   }
// // //   if ((acceleration.y <= 0.1) && (acceleration.y >= -0.1)) {
// // //     acceleration.y = 0;
// // //   }
// // //   if ((acceleration.z <= 0.1) && (acceleration.z >= -0.1)) {
// // //     acceleration.z = 0;
// // //   }
// // //   if ((magnetic_field.x <= 0.1) && (magnetic_field.x >= -0.1)) {
// // //     magnetic_field.x = 0;
// // //   }
// // //   if ((magnetic_field.y <= 0.1) && (magnetic_field.y >= -0.1)) {
// // //     magnetic_field.y = 0;
// // //   }
// // //   if ((magnetic_field.z <= 0.1) && (magnetic_field.z >= -0.1)) {
// // //     magnetic_field.z = 0;
// // //   }
// #include <Wire.h>

// void setup()
// {
//   Wire.setSCL(PB6);
//   Wire.setSDA(PB7);
//   Wire.begin();

//   Serial.begin(9600);
//   while (!Serial);             // Leonardo: wait for serial monitor
//   Serial.println("\nI2C Scanner");
// }

// void loop()
// {
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for(address = 1; address < 127; address++ )
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.print(address,HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error==4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.println(address,HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");

//   delay(5000);           // wait 5 seconds for next scan
// }
#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;

void setup()
{
  Serial.begin(9600);
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.begin();
  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0, 0);
}

void loop()
{
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180 / M_PI;

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

  delay(100);
}
