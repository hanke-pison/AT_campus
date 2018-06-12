#include "HP_GPS.h"

namespace HP {
  void GPS::begin() {
    ss.begin(gps_baudrate);
  }

  void GPS::update_gps() {
    while(ss.available()) {
      tiny_gps.encode(ss.read());
    }
    valid = tiny_gps.location.isValid();
    update_location();
  }

  void GPS::update_location() {
    if(tiny_gps.location.isValid()) {
      latitude = tiny_gps.location.lat();
      longitude = tiny_gps.location.lng();
      valid = true;
    } else {
      latitude = 999999;
      longitude = 999999;
      valid = false;
    }
  }
  
}

