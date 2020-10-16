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
	ros::init(argc, argv, "kinovoper");		
	ros::NodeHandle n_h;
	
	std::string namespc = argv[1];
	std::string teletopic = namespc + "/kinovaOper";

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
					puts("Joint 1 Right");
					break;
				case KEYCODE_R:
					code = 2;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 1 Left");
					break;
				case KEYCODE_U:
					code = 3;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 2 Right");
					break;
				case KEYCODE_D:
					code = 4;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 2 Left");
					break;
				case 'w':
					code = 5;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 3 Right");
					break;
				case 's':
					code = 6;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 3 Left");
					break;
				case 'd':
					code = 7;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 4 Right");
					break;
				case 'a':
					code = 8;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 4 Left");
					break;
				case 'p':
					code = 9;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 5 Right");
					break;
				case 'o':
					code = 10;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 5 Left");
					break;
				case 'l':
					code = 11;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 6 Right");
					break;
				case 'k':
					code = 12;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 6 Left");
					break;
				case 'm':
					code = 13;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 7 Right");
					break;
				case 'n':
					code = 14;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Joint 7 Left");
					break;
				case '1':
					code = 15;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Gripper Open");
					break;
				case '2':
					code = 16;
					msg.data = code;
					my_key_publisher.publish(msg);
					puts("Gripper Closed");
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











