#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Empty.h"
#include <ecl/threads.hpp>

ros::Duration d(0.01);
int fd;
using namespace ecl;

Mutex mutex;

extern "C" {
#include "movimento2.h"
};

class Listener
{
public:

	void reset_enc(const std_msgs::Empty::ConstPtr& msg){
		//Reseta os enconders
		mutex.lock();
		resetencoders(fd);
		mutex.unlock();
		ROS_INFO("ENCODER RESETADO");
		d.sleep();
	}

	void move(const geometry_msgs::Twist::ConstPtr& msg)
	{
		ROS_INFO ("Velocidade: linear.x=%f e angular.z=%f", msg->linear.x, msg->angular.z);
		if((msg->angular.z < 0.05)&&(msg->angular.z>=0)&&(msg->linear.x < 0.05)&&(msg->linear.x>=0)){
			mutex.lock();
			stopmotors(fd);
			mutex.unlock();
		}
		else if((msg->angular.z < 0.05)&&(msg->angular.z>=0) ){
        		mutex.lock();
			drivemotors(msg->linear.x*20, msg->linear.x*20, fd);
			mutex.unlock();
		}
		else if((msg->linear.x < 0.05)&&(msg->linear.x>=0)){
			mutex.lock();
			rotate(msg->angular.z*10, fd);
			mutex.unlock();
		}
		else if(msg->linear.x*8.5 != msg->angular.z*5){
			mutex.lock();
			drivemotors(msg->linear.x*17, msg->angular.z*10, fd);
			mutex.unlock();
		}
		d.sleep();
	}
};

int main (int argc, char **argv)
{

	ros::init(argc, argv, "forward");
	ros::NodeHandle n;
	fd = open_i2c_bus();
	Listener l;
	ros::Subscriber sub1 = n.subscribe("cmd_vel", 1, &Listener::move, &l);
	ros::Subscriber sub2 = n.subscribe("/reset_enc", 1, &Listener::reset_enc, &l);
	ros::Publisher pub = n.advertise<geometry_msgs::Pose2D>("/odom", 1);

	ros::AsyncSpinner s(2);
	s.start();

	ros::Rate r(100);
	ros::spinOnce();

	geometry_msgs::Pose2D odom;
	float linear;
	float angular;

	while(ros::ok()){

		mutex.lock();
		linear = readencoders(1, fd);
		mutex.unlock();
		mutex.lock();
		angular = readencodersang(1, fd);
		mutex.unlock();

		odom.x = linear;
		odom.theta = angular;
		pub.publish(odom);
		r.sleep();

	}

}
