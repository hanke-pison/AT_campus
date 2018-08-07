#include <ros.h>
#include <std_msgs/Int16.h>
#include "HP_Receiver.h"

// 1200-1700
// 1000-1900

#define turn_pin  A3 // Channel 4
#define brake_pin A2 // Channel 1
#define speed_pin A1 // Channel 3

HP::Receiver receiver(turn_pin, brake_pin, speed_pin);

void setup() {
  receiver.begin();
}

void loop() {
  receiver.spinOnce();
}
