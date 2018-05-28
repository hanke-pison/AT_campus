#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

static struct termios oldt;

void restore_terminal_settings(void)
{
    tcsetattr(0, TCSANOW, &oldt);  /* Apply saved settings */
}

void disable_waiting_for_enter(void)
{
    struct termios newt;

    /* Make terminal read 1 char at a time */
    tcgetattr(0, &oldt);  /* Save terminal settings */
    newt = oldt;  /* Init new settings */
    newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
    tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
    atexit(restore_terminal_settings); /* Make sure settings will be restored when program ends  */
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle n;
    ros::Publisher speed_pub = n.advertise<std_msgs::Int16>("speed", 1000);
    ros::Publisher brake_pub = n.advertise<std_msgs::Int16>("brake",1000);
    ros::Publisher turn_pub = n.advertise<std_msgs::Int16>("turn",1000);
    //ros::Rate loop_rate(10);
    int ch;
    //int speed_tmp=0;
    std_msgs::Int16 speed;
    std_msgs::Int16 brake;
    std_msgs::Int16 turn;
    speed.data=0;
    brake.data=100;
    turn.data=0;
    speed_pub.publish(speed);
    brake_pub.publish(brake);
    turn_pub.publish(turn);

    disable_waiting_for_enter();

    /* Key reading loop */
    while (1) {
        ch = getchar();
	if(ch==27){
	    ch = getchar();
	    ch = getchar();
	}
	switch (ch){
	    case 65:
		//printf("You pressed up \n");
		brake.data=0;
		speed.data<100?(speed.data += 10):(printf("Reached max. Speed \n"));
		//printf("Speed = %d\n", speed);
		break;
	    case 66:
                //printf("You pressed down \n");
		brake.data=0;
                speed.data>0?(speed.data -= 10):(printf("Stoped\n"));
		//printf("Speed = %d\n", speed);
		break;
	    case 67:
		turn.data>-300?(turn.data-=10):(printf("Right Turn Max.\n"));
                //printf("You pressed right \n");
                break;
	    case 68:
		turn.data<300?(turn.data+=10):(printf("Left Turn Max.\n"));
                //printf("You pressed left \n");
                break;
	    case 'b':
		speed.data = 0;
		printf("BRAKE!!! \n");
		brake.data=100;
		//printf("Speed = %d\n", speed);
		break;
	    case 'c':
                turn.data = 0;
                printf("Back To CENTER!!! \n");
                break;
	}
        //if (ch == 65) printf("You pressed up \n");
	//speed.data=speed_tmp;
	printf("Speed = %d, Turn = %d\n", speed.data, turn.data);
        //ROS_INFO("%d", speed.data);
	speed_pub.publish(speed);
	brake_pub.publish(brake);
	turn_pub.publish(turn);
	ros::spinOnce();
	if (ch == 'Q') break;  /* Press 'Q' to quit program */
    	//printf("you have pressed %d", ch);
	//loop_rate.sleep();
    }
//    printf("Quit!\n");
//    printf("Set Speed to 0. Brake!!! \n");
    speed.data=0;
    speed_pub.publish(speed);
//    ros::spinOnce();
    printf("Quit!\n");
    printf("Set Speed to 0. Brake!!! \n");
    ros::Duration(0.5).sleep();
    return 0;
}
