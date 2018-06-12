#ifndef HP_Receiver
#define HP_Receiver

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <ros.h>
#include <std_msgs/Int16.h>

namespace HP {
  class Receiver {
    public:
      Receiver(uint8_t turn_pin, uint8_t brake_pin, uint8_t speed_pin);
      void begin();
      void spinOnce();
      
    private:
      ros::NodeHandle nh;
      
      std_msgs::Int16 turn_msg;
      std_msgs::Int16 brake_msg;
      std_msgs::Int16 speed_msg;
      
      ros::Publisher turn_pub = ros::Publisher("turn_cmd", &turn_msg);
      ros::Publisher brake_pub = ros::Publisher("brake_cmd", &brake_msg);
      ros::Publisher speed_pub = ros::Publisher("speed_cmd", &speed_msg);
      
      uint8_t _turn_pin, _brake_pin, _speed_pin;
      int16_t turn_value, brake_value, speed_value, last_turn_value = 0;

      void init_ros();
      void setup_pin();
      void read_pin();
      void check_boundary();
      void publish_value(); 
  };
}


#endif
