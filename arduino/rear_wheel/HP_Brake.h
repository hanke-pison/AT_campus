#ifndef HP_BRAKE
#define HP_BRAKE

// For Arduino 1.0 and earlier
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define deg_to_min(deg) (deg * 60)
#define min_to_deg(mins) (mins / 60)

namespace HP {
  class Brake{    
    public:
      void begin();
      void end();
      
      uint16_t get_pulse_max();
      uint16_t get_pulse_min();
      int16_t get_rotation_start();
      int16_t get_rotation_end();
      
      void set_pulse_range(uint16_t pulse_min_us, uint16_t pulse_max_us);
      void set_rotation_range(int16_t rotation_start_min, int16_t rotation_end_min);

      int16_t read();
      uint8_t read_ocr();
      uint16_t read_pulse();
      int16_t read_rotation();
      int16_t read_rotation_min();
      
      void write(int16_t rotation_deg);
      void write_ocr(uint8_t ocr);
      void write_pulse(uint16_t pulse_us);
      void write_rotation(int16_t rotation_deg);
      void write_rotation_min(int16_t rotation_min);
      
    private:
      // Note: This class also need pin 11, so do not use pin 11 as other usage
      uint8_t _pin = 3;
      uint8_t _output_ocr_max = 156;
      uint16_t _output_pulse_max_us = 20000, _pulse_max_us = 2250, _pulse_min_us = 750;
      int16_t  _rotation_start_min = -5400, _rotation_end_min = 5400;
      enum Rotation_Direction {CCW, CW} _rotation_direction = CW;
      
      uint8_t pulse_to_ocr(uint16_t pulse_us);
      uint16_t ocr_to_pulse(uint8_t ocr);
      int16_t pulse_to_rotation(uint16_t pulse_us);
      uint16_t rotation_to_pulse(int16_t rotation_min);
      uint8_t rotation_to_ocr(int16_t rotation_min);
      int16_t ocr_to_rotation(uint8_t ocr);
  };
}
#endif
