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
using namespace ecl, std;
float wheel_distance = 0.25;
float wheel_radius = 0.05;
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

	float calculate_body_turn_radius(float linear_speed, d angular_speed){
		float body_turn_radius;
		if((angular_speed >= 0.0) && (angular_speed) < 0.02)
			body_turn_radius = 0.0;
		else
			body_turn_radius = linear_speed/angular_speed;
		return body_turn_radius;
	}

	float calculate_wheel_turn_radius(float body_turn_radius, float angular_speed, const string& wheel){

		float wheel_sign;
		float wheel_turn_radius;
		if(body_turn_radius != 0){
			if(wheel == "right")
				wheel_sign = 1.0;
			else if (wheel == "left")
				wheel_sign = -1.0;
			wheel_turn_radius = body_turn_radius+(wheel_sign*(wheel_distance/2));
		}
		else
			wheel_turn_radius = 0;
		return wheel_turn_radius;

	}

	float calculate_wheel_rpm(float linear_speed, float angular_speed, float wheel_turn_radius){

		float wheel_rpm;
		if (wheel_turn_radius != 0)
			wheel_rpm = (angular_speed*wheel_turn_radius)/wheel_radius;
		else
			if((linear_speed >= 0)&&(linear_speed < 0.02))
				wheel_rpm = 0;
			else
				wheel_rpm = linear_speed/wheel_radius;
		return whee_rpm;

	}

	void move(const geometry_msgs::Twist::ConstPtr& msg)
	{

		float body_turn_radius, right_wheel_turn_radius, left_wheel_turn_radius;
		string = wheel;

		ROS_INFO ("Velocidade: linear.x=%f e angular.z=%f", msg->linear.x, msg->angular.z);
		body_turn_radius = calculate_body_turn_radius(msg->linear.x, msg->angular.z);

		wheel = "right";
		right_wheel_turn_radius = calculate_wheel_turn_radius(body_turn_radius, angular_speed, wheel);

		wheel = "left";
                left_wheel_turn_radius = calculate_wheel_turn_radius(body_turn_radius, angular_speed, wheel);

		right_wheel_rpm = calculate_wheel_rpm(msg->linear.x, msg->angular.z, right_wheel_turn_radius);
		left_wheel_rpm = calculate_wheel_rpm(msg->linear.x, msg->angular.z, left_wheel_turn_radius);

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

	ros::Rate r(1);
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
