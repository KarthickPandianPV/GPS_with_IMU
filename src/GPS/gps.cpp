#include "gps.hpp"

#include <Arduino.h>

namespace gps_with_imu {
gps::gps() {}
gps::~gps() {}

void gps::initialize() {
  gps_serial_.begin(9600);
  delay(1000);
}

}  // namespace gps_with_imu