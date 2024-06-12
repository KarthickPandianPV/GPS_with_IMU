#include <Arduino.h>
#include <SoftwareSerial.h>
#include <string.h>

#define GPS_TX_PIN 1
#define GPS_RX_PIN 0
String sendMessage;
String receivedMessage;
String gprmc_msg;
// float latitude_conv;
char latitude_string[12];
char longitude_string[13];
char latitude_direction, longitude_direction;
double latitude, longitude, latitude_in_deg, longitude_in_deg;
double latitude_of_target=12.9453, longitude_of_target=80.2122, distance;
int i, j;
SoftwareSerial Serial1(GPS_RX_PIN, GPS_TX_PIN);
using namespace std;

void setup() {
  Serial.begin(9600);   // Initialize the Serial monitor for debugging
  Serial1.begin(9600);  // Initialize Serial1 for sending data
  delay(2000);
  // Serial1.println("Enter target latitude :");
  // latitude_of_target=Serial.parseFloat();
  // Serial1.println("Enter target longitude :");
  // longitude_of_target=Serial.parseFloat();

  Serial1.println("$PSTMWARM");
  delay(20000);
  // Serial.println("Cold start done.");
  // delay(1000);
  // Serial1.println("$PSTMINITGPS,1256.71812,N,08012.73105,E,0530,10,06,2024,09,51,00");
}

void loop() {
  while (Serial1.available() > 0) {
    
    char receivedChar = Serial1.read();
    if (receivedChar == '\n') {
      if ((receivedMessage[1] == 'G') && (receivedMessage[2] == 'P') && (receivedMessage[3] == 'R') && (receivedMessage[4] == 'M') && (receivedMessage[5] == 'C'))
        gprmc_msg = receivedMessage;
      receivedMessage = "";  // Reset the received message
      // Serial.println(gprmc_msg);
      if (gprmc_msg[18] == 'A') {
        for (i = 20; i <= 29; i++) {
          latitude_string[i - 20] = gprmc_msg[i];
        }
        latitude_direction = gprmc_msg[31];
        for (j = 33; j <= 43; j++) {
          longitude_string[j - 33] = gprmc_msg[j];
        }
        longitude_direction = gprmc_msg[45];
        // latitude_conv=convToFloat(latitude_string);
        latitude = atof(latitude_string);
        longitude = atof(longitude_string);
        Serial.print("Latitude: ");
        Serial.print(latitude, 4);
        Serial.print(",");
        Serial.print(latitude_direction);
        Serial.print("   Longitude: ");
        Serial.print(longitude, 4);
        Serial.print(",");
        Serial.println(longitude_direction);
        // Serial.print("Latitude_str: ");
        // Serial.println(latitude_string);
        // Serial.print("Longitude_str: ");
        // Serial.println(longitude_string);

        latitude_in_deg = (int)latitude / 100;
        longitude_in_deg = (int)longitude / 100;

        latitude_in_deg = latitude_in_deg + (latitude - 100.00 * latitude_in_deg) / 60.00;
        longitude_in_deg = longitude_in_deg + (longitude - 100.00 * longitude_in_deg) / 60.00;

        if (latitude_direction == 'S')
          latitude_in_deg *= (-1);
        if (longitude_in_deg == 'W')
          longitude_in_deg *= (-1);
        Serial.print("Latitude in degree: ");
        Serial.print(latitude_in_deg, 4);
        // Serial.print(",");
        // Serial.println(latitude_direction);
        Serial.print("   Longitude in degree: ");
        Serial.println(longitude_in_deg, 4);
        // Serial.print(",");
        // Serial.println(longitude_direction);
        // float lat1 = 12.9524976;
        // float long1 = 80.2165202;
        // float lat2 = -5.2473905;
        // float long2 = 44.6226517;

        // call the distance function
        double distance = getDistance(latitude_in_deg, longitude_in_deg, latitude_of_target, longitude_of_target);
        Serial.print("distance(in kms) is : ");
        Serial.println(distance, 4);
        double heading = getDirection(latitude_in_deg, longitude_in_deg, latitude_of_target, longitude_of_target);
        Serial.print("heading(in degrees) is : ");
        Serial.println(heading, 4);
        Serial.print('\n');
      } else
        Serial.println("GPS data invalid..!");
    } else
      receivedMessage += receivedChar;  // Append characters to the received message
  
  }
}

double getDistance(double latitude_in_deg, double longitude_in_deg,
                   double latitude_of_target, double longitude_of_target) {
  // Convert the latitudes
  // and longitudes
  // from degree to radians.
  latitude_in_deg = toRadians(latitude_in_deg);
  longitude_in_deg = toRadians(longitude_in_deg);
  latitude_of_target = toRadians(latitude_of_target);
  longitude_of_target = toRadians(longitude_of_target);

  // Haversine Formula
  double delta_long = longitude_of_target - longitude_in_deg;
  double delta_lat = latitude_of_target - latitude_in_deg;

  double distance = pow(sin(delta_lat / 2), 2) + cos(latitude_in_deg) * cos(latitude_of_target) * pow(sin(delta_long / 2), 2);

  distance = 2 * asin(sqrt(distance));

  // Radius of Earth in
  // Kilometers, R = 6371
  // Use R = 3956 for miles
  double R = 6371;

  // Calculate the result
  distance = distance * R;
  return distance;
}

int getDirection(double latitude_in_deg, double longitude_in_deg, double latitude_of_target, double longitude_of_target) {
  double current_lat_in_rad = toRadians(latitude_in_deg);
  double target_lat_in_rad = toRadians(latitude_of_target);
  double current_long_in_rad = toRadians(longitude_in_deg);
  double target_long_in_rad = toRadians(longitude_of_target);
  double Y = sin(target_long_in_rad - current_long_in_rad) * cos(target_lat_in_rad);
  double X = cos(current_lat_in_rad) * sin(target_lat_in_rad) - sin(current_lat_in_rad) * cos(target_lat_in_rad) * cos(target_long_in_rad - current_long_in_rad);
  double degree = toDegrees(atan2(Y, X));
  // note that this implementation doesn't use the module, but angles lower than 0 get augmented by 360 only
  if (degree < 0) {
    degree = 360 + degree;
  }
  double angle = degree;
  int a = (int)(abs(angle) + (1 / 7200));
  return a;
}

double toRadians(double angle) {
  return (PI / 180) * angle;
}

double toDegrees(double rad) {
  return (rad * 180) / PI;
}