#ifndef HP_GPS
#define HP_GPS

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

namespace HP {
  class GPS {
    public:
      double latitude, longitude;
      bool valid = false;
      void begin();

      void update_gps();
      void update_location();
      
    private:
      const uint32_t gps_baudrate = 9600;
      SoftwareSerial ss = SoftwareSerial(2, 3);
      TinyGPSPlus tiny_gps;
  };
}

#endif
