#include "HP_IMU.h"
#include "HP_GPS.h"

HP::IMU imu;
HP::GPS gps;

void setup() {
  imu.begin();
  gps.begin();
}

void loop() {
  imu.update_imu();
  gps.update_gps();
}
