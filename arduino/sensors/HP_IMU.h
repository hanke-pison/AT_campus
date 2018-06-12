#ifndef HP_IMU
#define HP_IMU

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP280.h"

namespace HP {
  class IMU {
    public:
      float ax, ay, az, gx, gy, gz, mx, my, mz, heading, tilt_heading, pressure, altitude, temperature;

      void begin();

      void update_acceleration();
      void update_rotation();
      void update_compass();
      void update_heading();
      void update_tilt_heading();
      void update_pressure();
      void update_altitude();
      void update_temperature();
      void update_imu();
      
      void calibrate_compass();
      
    private:
      I2Cdev I2C_M;
      MPU9250 mpu;
      BMP280 bmp;
      float mx_centre = 0, my_centre = 0, mz_centre = 0;
  };
}
#endif
