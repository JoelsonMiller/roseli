#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread.hpp"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Empty.h"
#include <ecl/threads.hpp>
#include <roseli/ResetEnc.h>
#include <roseli/GetOdom.h>

ros::Duration d(0.01);
int fd;
using namespace ecl;
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

	float calculate_body_turn_radius(float linear_speed, float angular_speed){
		float body_turn_radius;
		if((angular_speed < 0.02)&&(angular_speed > -0.02))
			body_turn_radius = 0.0;
		else
			body_turn_radius = linear_speed/angular_speed;
		return body_turn_radius;
	}

	float calculate_wheel_turn_radius(float body_turn_radius, float angular_speed, const std::string& wheel){

		float wheel_sign;
		float wheel_turn_radius;
		if(body_turn_radius != 0){
			if(wheel == "right")
				wheel_sign = 1.0;
			else if (wheel == "left")
				wheel_sign = -1.0;
			wheel_turn_radius = body_turn_radius+(wheel_sign*(wheel_distance/2));
		}
		else if(angular_speed != 0){
			if(angular_speed > 0){
				if(wheel == "right")
					wheel_turn_radius = 1;
				else
					wheel_turn_radius = 0;
			}
			else if(angular_speed < 0){
				if(wheel == "left")
					wheel_turn_radius = 1;
				else
					wheel_turn_radius = 0;
				}
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
			if((linear_speed < 0.02)&&(linear_speed > -0.02))
				wheel_rpm = 0;
			else
				wheel_rpm = linear_speed / wheel_radius;
		return wheel_rpm;

	}

	void move(const geometry_msgs::Twist::ConstPtr& msg)
	{

		float body_turn_radius, right_wheel_turn_radius, left_wheel_turn_radius, left_wheel_rpm, right_wheel_rpm;
		std::string wheel;

		ROS_INFO ("Velocidade: linear.x=%f e angular.z=%f", msg->linear.x, msg->angular.z);
		body_turn_radius = calculate_body_turn_radius(msg->linear.x, msg->angular.z);

		wheel = "right";
		right_wheel_turn_radius = calculate_wheel_turn_radius(body_turn_radius, msg->angular.z, wheel);

		wheel = "left";
                left_wheel_turn_radius = calculate_wheel_turn_radius(body_turn_radius, msg->angular.z, wheel);

		right_wheel_rpm = calculate_wheel_rpm(msg->linear.x, msg->angular.z, right_wheel_turn_radius);
		left_wheel_rpm = calculate_wheel_rpm(msg->linear.x, msg->angular.z, left_wheel_turn_radius);

		right_wheel_rpm = right_wheel_rpm*128/170;
		left_wheel_rpm = left_wheel_rpm*128/170;

		if((right_wheel_rpm == 0) && (left_wheel_rpm == 0))
			stopmotors(fd);
		else if(right_wheel_rpm == 0)
			rotate(left_wheel_rpm, fd);
		else if(left_wheel_rpm == 0)
			rotate(right_wheel_rpm, fd);
		else
			drivemotors(right_wheel_rpm, left_wheel_rpm, fd);

		d.sleep();
	}


};

bool server_reset_enconder(roseli::ResetEnc::Request &req, roseli::ResetEnc::Response &res){
		//Reseta os enconders
		mutex.lock();
		resetencoders(fd);
		mutex.unlock();
		ROS_INFO("ENCODER RESETADO");
		res.reseted = true;
		return true;
	}

bool server_get_odom(roseli::GetOdom::Request &req, roseli::GetOdom::Response &res){
		float linear, angular;

		mutex.lock();
		linear = readencoders(1, fd);
		mutex.unlock();
		mutex.lock();
		angular = readencodersang(1, fd);
		mutex.unlock();

		res.dist.x = linear;
		res.dist.y = 0;
		res.dist.theta = angular;
		return true;
	}

int main (int argc, char **argv)
{
	std::String s_odom;
	std::String s_reset;
	ros::init(argc, argv, "forward");
	ros::NodeHandle n;
	n.param("get_odom_stream", s_odom, "odom");
	n.param("reset_enc_stream", s_reset, "odom");
	fd = open_i2c_bus();
	Listener l;
	ros::Subscriber sub1 = n.subscribe("cmd_vel", 1, &Listener::move, &l);
	
	
	if(s_odom == "odom")
		ros::Publisher pub = n.advertise<geometry_msgs::Pose2D>("/odom", 1);
	else if(s_dom == "server")
		ros::ServiceServer odom_server = n.advertiseService("/odom_server", server_get_odom);
	else
		ROS_INFO("Parameters 'get_odom_stream' has invalid input");
	
	if(s_reset == "odom")
		ros::Subscriber sub2 = n.subscribe("/reset_enc", 1, &Listener::reset_enc, &l);
	else if(s_reset == "server")
		ros::ServiceServer reset_enc_server = n.advertiseService("/reset_enc_server", server_reset_enconder);
	else
		ROS_INFO("Parameters 'reset_enc_stream' has invalid input");
	
	ros::AsyncSpinner s(2);
	s.start();

	ros::Rate r(5);
	ros::spinOnce();

	if(s_odom == "odom"){
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
			odom.y = 0;
			odom.theta = angular;
			pub.publish(odom);
			r.sleep();

		}
	}
}

