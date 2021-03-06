
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

extern "C" {
#include "movimento.h"
};

void mensagemrecebida(const geometry_msgs::Twist::ConstPtr& msg)
{
	ROS_INFO ("Eu escutei: %f", msg->linear.x);
	if((msg->angular.z == 0)&&(msg->linear.x < 0)){
		stopmotors();
		usleep(1);
	}
	else if(msg->angular.z < 0.05 ){
        	drivemotors(msg->linear.x*20, msg->linear.x*20);
        	usleep(1);
	}
	else if(msg->linear.x == 0){
		rotate(msg->angular.z*10);
		usleep(1);
	}
	else if(msg->linear.x*8.5 != msg->angular.z*5){
		drivemotors(msg->linear.x*17, msg->angular.z*10);
		usleep(1);

	}
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "forward");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe ("cmd_vel", 1, mensagemrecebida);
//	ros::Publisher pub = node.advertise<std_msgs::Float32>("encoder_ang", 10);

//	std_msgs::Float32 ang;

//	while(1){
//		ang = readencondersang();
//
	//}
	ros::spin();
	//return 0;
}
