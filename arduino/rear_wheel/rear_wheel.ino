#include "HP_DrivingMotor.h"
#include "HP_Brake.h"

HP::DrivingMotor driving_motor;
HP::Brake brake_servo;

#define voltage_min_mv 1400
#define voltage_max_mv 3800
#define pulse_min_us 800
#define pulse_max_us 1600

#define speed_max_cmd 100
#define brake_max_cmd 100

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void speed_cb(const std_msgs::Int16& cmd_msg){
  if(cmd_msg.data != 0){
    driving_motor.write_voltage(map(cmd_msg.data, 1, speed_max_cmd, voltage_min_mv, voltage_max_mv));
  } else {
    driving_motor.write_ocr(0);
  }
}

void brake_cb(const std_msgs::Int16& cmd_msg){
  brake_servo.write(cmd_msg.data);
}

ros::Subscriber<std_msgs::Int16> speed_sub("speed_cmd", speed_cb);
ros::Subscriber<std_msgs::Int16> brake_sub("brake_cmd", brake_cb);

void setup() {
  nh.initNode();
  nh.subscribe(speed_sub);
  nh.subscribe(brake_sub);
  driving_motor.begin();
  driving_motor.set_voltage_range(voltage_min_mv, voltage_max_mv);
  brake_servo.begin();
  brake_servo.set_rotation_range(0, brake_max_cmd);
  brake_servo.set_pulse_range(pulse_min_us, pulse_max_us);
}

void loop() {
  nh.spinOnce();
}
