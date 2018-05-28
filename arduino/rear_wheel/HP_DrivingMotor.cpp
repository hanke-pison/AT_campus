#include "HP_DrivingMotor.h"

namespace HP {
  void DrivingMotor::begin() {
    // setup _pin as pwm  with ___ Hz
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS10);
  }

  void DrivingMotor::end() {
    // close
    write_ocr(0);
  }

  uint16_t DrivingMotor::get_voltage_max() {
    return _voltage_max_mv;
  }
  
  uint16_t DrivingMotor::get_voltage_min() {
    return _voltage_min_mv;
  }
  
  void DrivingMotor::set_voltage_range(uint16_t voltage_min_mv, uint16_t voltage_max_mv) {
    if(voltage_min_mv < voltage_max_mv) {
      _voltage_min_mv = voltage_min_mv;
      _voltage_max_mv = voltage_max_mv;
    } else {
      _voltage_min_mv = voltage_max_mv;
      _voltage_max_mv = voltage_min_mv;
    }
    _voltage_max_mv = (_voltage_max_mv <= _output_voltage_max_mv) ? _voltage_max_mv : _output_voltage_max_mv;
  }
  
  uint16_t DrivingMotor::read_voltage() {
    return ocr_to_voltage(OCR1A);
  }
  
  void DrivingMotor::write_ocr(uint16_t ocr) {
    if(ocr == 0) {
      digitalWrite(_pin, LOW);
    } else if(ocr >= _output_ocr_max) {
      digitalWrite(_pin, HIGH);
    } else {
      TCCR1A |= _BV(COM1A1);
      OCR1A = ocr;
    }
  }

  void DrivingMotor::write_duty(uint16_t duty_permille) {
    duty_permille = (duty_permille <= 1000) ? duty_permille : 1000;
    write_ocr(map(duty_permille, 0, 1000, 0, _output_ocr_max));
  }
    
  void DrivingMotor::write_voltage(uint16_t voltage_mv) {
    voltage_mv = (voltage_mv >= _voltage_max_mv) ? _voltage_max_mv : ((voltage_mv <= _voltage_min_mv) ? _voltage_min_mv : voltage_mv);
    write_ocr(voltage_to_ocr(voltage_mv));
  }

  // private

  uint16_t DrivingMotor::voltage_to_ocr(uint16_t voltage_mv) {
    return map(voltage_mv, 0, _output_voltage_max_mv, 0, _output_ocr_max);
  }

  uint16_t DrivingMotor::ocr_to_voltage(uint16_t ocr) {
    return map(ocr, 0, _output_ocr_max, 0, _output_voltage_max_mv);
  }
}
