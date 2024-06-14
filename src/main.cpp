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
double latitude_of_target = 12.9453, longitude_of_target = 80.2122, distance;
int i, j;
SoftwareSerial Serial1(GPS_RX_PIN, GPS_TX_PIN);
using namespace std;

void setup() {}

void loop() {}