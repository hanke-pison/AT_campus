#include "HP_Stepper.h"

HP::Stepper turn_stepper(8, 9, 10, 11, 12);
#define ppr 25000   // pulses per revolution
#define ppd ppr/360 // pulses per degree

long count = 0;
long target_count = 0;
bool active = false;

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

void turn_cb(const std_msgs::Int16& cmd_msg){
  //cmd_msg ranges from -300 to 300 (i.e. -30 degree to 30 degree)
  target_count = -((float)(cmd_msg.data/10))*ppd;
}

ros::Subscriber<std_msgs::Int16> turn_sub("turn", turn_cb);

void setup() {
  turn_stepper.init();
  nh.initNode();
  nh.subscribe(turn_sub);
}

unsigned int time_us = 200;

void loop() {
  nh.spinOnce();
  if(count < target_count) {
    active = true;
    turn_stepper.ccw(time_us);
    count++;
  } else if (count > target_count) {
    active = true;
    turn_stepper.cw(time_us);
    count--;
  } else {
    if(active) {
      active = false;
      turn_stepper.pause();
    }
  }
}
