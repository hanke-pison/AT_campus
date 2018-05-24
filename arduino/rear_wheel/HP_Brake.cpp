#include "HP_Brake.h"

namespace HP {
  void Brake::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20);           //
    OCR2A = _output_ocr_max;
    OCR2B = 4;
  }
  
  void Brake::end() {
    
  }

  uint16_t Brake::get_pulse_max() {
    return _pulse_max_us;
  }
  
  uint16_t Brake::get_pulse_min() {
    return _pulse_min_us;
  }
  
  int16_t Brake::get_rotation_start() {
    return _rotation_start_min;
  }
  
  int16_t Brake::get_rotation_end() {
    return _rotation_end_min;
  }

  void Brake::set_pulse_range(uint16_t pulse_min_us, uint16_t pulse_max_us) {
    if(pulse_min_us < pulse_max_us) {
      _pulse_min_us = pulse_min_us;
      _pulse_max_us = pulse_max_us;
    } else {
      _pulse_min_us = pulse_max_us;
      _pulse_max_us = pulse_min_us;
    }
    _pulse_max_us = (_pulse_max_us < _output_pulse_max_us) ? _pulse_max_us : _output_pulse_max_us;
  }

  void Brake::set_rotation_range(int16_t rotation_start_min, int16_t rotation_end_min) {
    _rotation_direction = (rotation_start_min < rotation_end_min) ? CW : CCW;
    _rotation_start_min = rotation_start_min;
    _rotation_end_min = rotation_end_min;
  }

  int16_t Brake::read() {
    return read_rotation();
  }
  
  uint8_t Brake::read_ocr() {
    return OCR2B;
  }
  
  uint16_t Brake::read_pulse() {
    return ocr_to_pulse(read_ocr());
  }
  
  int16_t Brake::read_rotation() {
    return min_to_deg(ocr_to_rotation(read_ocr()));
  }
  
  int16_t Brake::read_rotation_min() {
    return ocr_to_rotation(read_ocr());
  }

  void Brake::write(int16_t rotation_deg) {
    write_rotation(rotation_deg);
  }
  
  void Brake::write_ocr(uint8_t ocr) {
    if(ocr == 0) {
      digitalWrite(_pin, LOW);
    } else if(ocr >= _output_ocr_max) {
      digitalWrite(_pin, HIGH);
    } else {
      TCCR2A |= _BV(COM2B1);
      OCR2B = ocr;
    }
  }
  
  void Brake::write_pulse(uint16_t pulse_us) {
    pulse_us = (pulse_us > _pulse_max_us) ? _pulse_max_us : (pulse_us < _pulse_min_us) ? _pulse_min_us : pulse_us;
    write_ocr(pulse_to_ocr(pulse_us));
  }
  
  void Brake::write_rotation(int16_t rotation_deg) {
    write_rotation_min(deg_to_min(rotation_deg));
  }
  
  void Brake::write_rotation_min(int16_t rotation_min) {
    switch(_rotation_direction) {
      case CCW :
        rotation_min = (rotation_min > _rotation_end_min) ? _rotation_end_min : (rotation_min < _rotation_start_min) ? _rotation_start_min : rotation_min;
        break;
      case CW :
        rotation_min = (rotation_min > _rotation_start_min) ? _rotation_start_min : (rotation_min < _rotation_end_min) ? _rotation_end_min : rotation_min;
        break;
    }
    write_ocr(rotation_to_ocr(rotation_min));
  }

  // private
  
  uint8_t Brake::pulse_to_ocr(uint16_t pulse_us) {
    return map(pulse_us, 0, _output_pulse_max_us, 0, _output_ocr_max);
  }
  
  uint16_t Brake::ocr_to_pulse(uint8_t ocr) {
    return map(ocr, 0, _output_ocr_max, 0, _output_pulse_max_us);
  }
  
  int16_t Brake::pulse_to_rotation(uint16_t pulse_us) {
    switch(_rotation_direction) {
      case CCW :
        return map(pulse_us, _pulse_min_us, _pulse_max_us, _rotation_start_min, _rotation_end_min);
        break;
      case CW :
        return map(pulse_us, _pulse_min_us, _pulse_max_us, _rotation_end_min, _rotation_start_min);
        break;
    }
  }
  
  uint16_t Brake::rotation_to_pulse(int16_t rotation_min) {
    switch(_rotation_direction) {
      case CCW :
        return map(rotation_min, _rotation_start_min, _rotation_end_min, _pulse_min_us, _pulse_max_us);
        break;
      case CW :
        return map(rotation_min, _rotation_end_min, _rotation_start_min, _pulse_min_us, _pulse_max_us);
        break;
    }
  }
  
  uint8_t Brake::rotation_to_ocr(int16_t rotation_min) {
    return pulse_to_ocr(rotation_to_pulse(rotation_min));
  }
  
  int16_t Brake::ocr_to_rotation(uint8_t ocr) {
    return pulse_to_rotation(ocr_to_pulse(ocr));
  }
  
  
}
