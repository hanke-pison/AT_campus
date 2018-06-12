#include "HP_Receiver.h"

namespace HP {
  Receiver::Receiver(uint8_t turn_pin, uint8_t brake_pin, uint8_t speed_pin)
 {
  
    _turn_pin = turn_pin;
    _brake_pin = brake_pin;
    _speed_pin = speed_pin;
  }
  
  void Receiver::begin() {
    init_ros();
    setup_pin();
  }

  void Receiver::spinOnce() {
    read_pin();
    check_boundary();
    publish_value(); 
    nh.spinOnce();
  }

  // private
  
  void Receiver::init_ros() {
    nh.initNode();
    nh.advertise(turn_pub);
    nh.advertise(brake_pub);
    nh.advertise(speed_pub);
  }

  void Receiver::setup_pin() {
    pinMode(_turn_pin, INPUT);
    pinMode(_brake_pin, INPUT);
    pinMode(_speed_pin, INPUT);
  }

  void Receiver::read_pin() {
    turn_value = pulseIn(_turn_pin, HIGH);
    brake_value = pulseIn(_brake_pin, HIGH);
    speed_value = pulseIn(_speed_pin, HIGH);
  }

  void Receiver::check_boundary() {
    int16_t turn_step = 50;
    turn_value = map(turn_value, 1300, 1700, 300, -300) / turn_step;
    turn_value = (abs(turn_value) <= 1) ? 0 : turn_value * turn_step;
    last_turn_value = turn_value;

    int16_t brake_tolerance = 100;
    brake_value = (abs(brake_value - 1500) > brake_tolerance) ? 100 : 0;
    
    speed_value = brake_value ? 0 : map(speed_value, 1000, 1900, 0, 100);
  }
  
  void Receiver::publish_value() {
    turn_msg.data = turn_value;
    brake_msg.data = brake_value;
    speed_msg.data = speed_value;
  
    turn_pub.publish(&turn_msg);
    brake_pub.publish(&brake_msg);
    speed_pub.publish(&speed_msg);
  }
}

