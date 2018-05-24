#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 turn_msg;
std_msgs::Int16 brake_msg;
std_msgs::Int16 speed_msg;

ros::Publisher turn_pub("turn", &turn_msg);
ros::Publisher brake_pub("brake", &brake_msg);
ros::Publisher speed_pub("speed", &speed_msg);

// 1200-1700
// 1000-1900

#define turn_pin  A3 // Channel 4
#define brake_pin A2 // Channel 1
#define speed_pin A1 // Channel 3


void setup() {
  nh.initNode();
  nh.advertise(turn_pub);
  nh.advertise(brake_pub);
  nh.advertise(speed_pub);
  pinMode(turn_pin, INPUT);
  pinMode(brake_pin, INPUT);
  pinMode(speed_pin, INPUT);
}

int turn_value, brake_value, speed_value;
int last_turn_value = 0;

void read_value() {
  turn_value = pulseIn(turn_pin, HIGH);
  brake_value = pulseIn(brake_pin, HIGH);
  speed_value = pulseIn(speed_pin, HIGH);
}

void check_boundary() {
  turn_value = map(turn_value, 1300, 1700, 300, -300);
  
  turn_value = turn_value/50;
  turn_value = turn_value*50;
  if(abs(turn_value) < 51){
    turn_value = 0;
  }
  
  last_turn_value = turn_value;
  
  speed_value = map(speed_value, 1000, 1900, 0, 100);

  if(abs(brake_value-1500)>100){
    brake_value = 100;
    speed_value = 0;
  } else {
    brake_value = 0;
  }
}

void publish_value() {
  turn_msg.data = turn_value;
  brake_msg.data = brake_value;
  speed_msg.data = speed_value;

  turn_pub.publish( &turn_msg);
  brake_pub.publish( &brake_msg);
  speed_pub.publish( &speed_msg);
}

void loop() {
  read_value();
  check_boundary();
  publish_value(); 
  nh.spinOnce();
}
