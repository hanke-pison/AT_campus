#ifndef HP_PWM
#define HP_PWM

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

namespace HP {
  class Stepper {
    public:
      Stepper(int pin, int cw_positive, int cw_negative, int ccw_positive, int ccw_negative);
      void set_time_min(unsigned int time_min_us);
      void init();
      void ccw(unsigned int time_us);
      void cw(unsigned int time_us);
      void pause();
    private:
      enum DIRECTION {ccw_direction, cw_direction, pause_direction};
      DIRECTION _turn_direction;
      int _pin, _ccw_positive, _ccw_negative, _cw_positive, _cw_negative;
      unsigned int _time_min_us;
      unsigned int check_time(unsigned int time_us);
  };
}

#endif
