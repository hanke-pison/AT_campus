#include "HP_Stepper.h"

namespace HP {
  Stepper::Stepper(int pin, int cw_positive, int cw_negative, int ccw_positive, int ccw_negative)
  : _pin(pin), _cw_positive(cw_positive), _cw_negative(cw_negative), _ccw_positive(ccw_positive), _ccw_negative(ccw_negative),
  _turn_direction(pause_direction), _time_min_us(100) {
    pinMode(_pin, OUTPUT);
    pinMode(_ccw_positive, OUTPUT);
    pinMode(_ccw_negative, OUTPUT);
    pinMode(_cw_positive, OUTPUT);
    pinMode(_cw_negative, OUTPUT);
  }

  void Stepper::set_time_min(unsigned int time_min_us) {
    _time_min_us = time_min_us;
  }

  void Stepper::init() {
    digitalWrite(_pin, LOW);
    pause();
  }

  void Stepper::ccw(unsigned int time_us) {
    if(_turn_direction == cw_direction){
      digitalWrite(_cw_negative, LOW);
      digitalWrite(_cw_positive, LOW);
      _turn_direction = ccw_direction;
    }
    time_us = check_time(time_us);
    digitalWrite(_ccw_negative, LOW);
    digitalWrite(_ccw_positive, HIGH);
    delayMicroseconds(time_us);
    digitalWrite(_ccw_negative, HIGH);
    digitalWrite(_ccw_positive, LOW);
    delayMicroseconds(time_us);
  }

  void Stepper::cw(unsigned int time_us) {
    if(_turn_direction == ccw_direction){
      digitalWrite(_ccw_negative, LOW);
      digitalWrite(_ccw_positive, LOW);
      _turn_direction = cw_direction;
    }
    time_us = check_time(time_us);
    digitalWrite(_cw_negative, LOW);
    digitalWrite(_cw_positive, HIGH);
    delayMicroseconds(time_us);
    digitalWrite(_cw_negative, HIGH);
    digitalWrite(_cw_positive, LOW);
    delayMicroseconds(time_us);
  }

  void Stepper::pause() {
    _turn_direction = pause_direction;
    digitalWrite(_ccw_negative, LOW);
    digitalWrite(_ccw_positive, LOW);
    digitalWrite(_cw_negative, LOW);
    digitalWrite(_cw_positive, LOW);
  }

  // private

  unsigned int Stepper::check_time(unsigned int time_us) {
    if(time_us < _time_min_us) {
      return _time_min_us;
    } else {
      return time_us;
    }
  }
}


