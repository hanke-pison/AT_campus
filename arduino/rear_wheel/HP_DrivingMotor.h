// Arduino HP PWM Library v1.1.0 - PWM outputs
// https://github.com/alextaujenis/RBD_Servo
// Copyright 2018 Hanson Chang

#ifndef HP_DrivingMotor
#define HP_DrivingMotor

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

namespace HP {  
  class DrivingMotor {
    public:
      void begin();
      void end();

      uint16_t get_voltage_max();
      uint16_t get_voltage_min();
      
      void set_voltage_range(uint16_t voltage_min_mv, uint16_t voltage_max_mv);

      uint16_t read_voltage();

      void write_ocr(uint16_t ocr);
      void write_duty(uint16_t duty_permille);
      void write_voltage(uint16_t voltage_mv);
      
    private:
      uint8_t _pin = 9;
      uint16_t _output_ocr_max = 1023, _output_voltage_max_mv = 5000, _voltage_max_mv = 5000, _voltage_min_mv = 0;
      uint16_t voltage_to_ocr(uint16_t voltage_mv);
      uint16_t ocr_to_voltage(uint16_t ocr);
  };
}
#endif
