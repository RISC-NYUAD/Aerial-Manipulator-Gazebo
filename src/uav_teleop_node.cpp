#include <iostream>
#include <termios.h>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <string>
#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42


using namespace std;

struct termios tempcopy, changed;

void keyloop(int, ros::Publisher&);
void prepare(int);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleoper");		
	ros::NodeHandle n_h;
	
	std::string namespc = argv[1];
	std::string teletopic = namespc + "/teleoperator";

	ros::Publisher key_pub = n_h.advertise<std_msgs::Int16>(teletopic, 1);


	prepare(0);
	keyloop(0,key_pub);
	
	tcsetattr(0, TCSANOW, &tempcopy);
	return 0;
}

void keyloop(int fd, ros::Publisher& my_key_publisher){
	
	ros::Rate loop_rate(10);
	int code = 0; 
	char c;
	std_msgs::Int16 msg;
	while(1){
		if(read(0,&c,1)<0){
			puts("error");
			break;
		}else{
			switch(c){
				case KEYCODE_L:
					code = 1;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Turning left");
					break;
				case KEYCODE_R:
					code = 2;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Turning right");
					break;
				case KEYCODE_U:
					code = 3;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Going up");
					break;
				case KEYCODE_D:
					code = 4;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Going down");
					break;
				case 'w':
					code = 5;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Going X forward");
					break;
				case 's':
					code = 6;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Going X backward");
					break;
				case 'd':
					code = 7;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Going Y forward");
					break;
				case 'a':
					code = 8;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Going Y backward");
					break;
				case 'b':
					code = 9;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Land");
					break;

			}

			loop_rate.sleep();		
		}

		if(c=='q'){break;}
		c='e'; //i.e. empty
	}	
	return;
}



void prepare(int fd){

	tcgetattr(fd, &tempcopy);
	memcpy(&changed, &tempcopy, sizeof(struct termios));
	changed.c_lflag &=~ (ICANON | ECHO);
	changed.c_cc[VEOL] = 1;
	changed.c_cc[VEOF] = 2;
	tcsetattr(fd, TCSANOW, &changed);

	puts("Reading from keyboard");
	puts("=====================");
	return;
}











