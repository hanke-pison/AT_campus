#include "HP_IMU.h"

namespace HP {
  void IMU::begin() {
    Wire.begin();
    mpu.initialize();
    bmp.init();
    //  calibrate_compass();
  }

  void IMU::update_acceleration() {
    int16_t x, y, z;
    mpu.getAcceleration(&x, &y, &z);
    ax = (float) x / 16384;
    ay = (float) y / 16384;
    az = (float) z / 16384;
  }

  void IMU::update_rotation() {
    int16_t x, y, z;
    mpu.getRotation(&x, &y, &z);
    gx = (float) x * 250 / 32768;
    gy = (float) y * 250 / 32768;
    gz = (float) z * 250 / 32768;
  }

  void IMU::update_compass() {
    int16_t x, y, z;
    uint8_t buffer_m[6];
    
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
    delay(10);
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    x = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    y = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    z = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    mx = (float) x * 1200 / 4096 - mx_centre;
    my = (float) y * 1200 / 4096 - my_centre;
    mz = (float) z * 1200 / 4096 - mz_centre;
  }

  void IMU::update_heading() {
    // make sure to update compass before this function is called
    // update_compass();
    heading = 180 * atan2(my, mx) / PI;
    if (heading < 0) heading += 360;
  }
  
  void IMU::update_tilt_heading() {
    // make sure to update acceleration and compass before this function is called
    // update_acceleration();
    // update_compass();
    float pitch = asin(-ax);
    float roll = asin(ay / cos(pitch));
    float xh = mx * cos(pitch) + mz * sin(pitch);
    float yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    float zh = -mx * cos(roll) * sin(pitch) + my * sin(roll) + mz * cos(roll) * cos(pitch);
    tilt_heading = 180 * atan2(yh, xh) / PI;
    if (yh < 0) tilt_heading += 360;
  }
  
  void IMU::update_pressure() {
    pressure = (float) bmp.getPressure() / 101325;
  }
   
  void IMU::update_altitude() {
    // make sure to update pressure before this function is called
    // update_pressure();
    altitude = (1.0 - pow(pressure, 1 / 5.25588)) / 0.0000225577;
  }

  void IMU::update_temperature() {
    temperature = bmp.getTemperature();
  }

  void IMU::update_imu() {
    update_rotation();
    update_acceleration();
    update_compass();
    update_heading()
    update_tilt_heading();
    update_pressure()
    update_altitude();
    update_temperature();
  }

  void IMU::calibrate_compass() {
    float mx_max, mx_min, my_max, my_min, mz_max, mz_min;
    mx_centre = 0;
    my_centre = 0;
    mz_centre = 0;
    for (int i=0; i<5000; i++) {
      update_compass();
      mx_max = (mx_max > mx) ? mx_max : mx;
      my_max = (my_max > my) ? my_max : my;
      mz_max = (mz_max > mz) ? mz_max : mz;
      mx_min = (mx_min < mx) ? mx_min : mx;
      my_min = (my_min < my) ? my_min : my;
      mz_min = (mz_min < mz) ? mz_min : mz;
    }
    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;
  }

}
