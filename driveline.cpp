#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "roseli/PointVector.h"
#include "geometry_msgs/Twist.h"

using namespace std;
geometry_msgs::Twist velocity;
ros::Publisher pub_vel;

void points_sub(const roseli_gazebo::PointVector::ConstPtr& points){

	double rightside = 0;
	double leftside = 0;
	double res;
	ros::Rate rate(0.5);
	ros::Rate later(0.2);
	if(points->points_center.size()!=0){

		leftside = abs(0 - points->points_center[0].x);
		rightside = abs(640 - points->points_center[1].x);
		res = leftside-rightside;
	}

//	cout<<res<<endl;
//	 ROS_INFO("Pontos na imagem %lu", points->points.size() );

	if((points->points_center.size()  == 1)||(points->points_center.size()==0)){
		//cout<<"Por quê!"<<endl;
		velocity.linear.x=0;
		velocity.angular.z=0;
	}else if(points->points_center.size() == 2){

		//cout<<"I am here"<<endl;
		velocity.linear.x=0.5;
		velocity.angular.z=res/125;

	}else if(/*(points->points.size()>2)&&(points->points.size()<5)*/ points->points_center.size()==4){
		if(res<25){
			ROS_INFO("Pontos na imagem %lu", points->points_center.size() );
			velocity.linear.x = 0.25;
                        velocity.angular.z = 0;
                        pub_vel.publish(velocity);
			rate.sleep();
			velocity.linear.x = 0;
			velocity.angular.z = -0.85;
			pub_vel.publish(velocity);
			rate.sleep();
			velocity.linear.x = 0.1;
                        velocity.angular.z = 0;
                        pub_vel.publish(velocity);
                        rate.sleep();
			velocity.linear.x=0;
                        velocity.angular.z=0;
                        pub_vel.publish(velocity);
                        later.sleep();
		}
	}

	pub_vel.publish(velocity);
}


int main(int argc, char** argv){

	ros::init(argc, argv, "Motor_Move");
	ros::NodeHandle node;
	ros::Subscriber sub_points=node.subscribe("line/points", 1, points_sub);
	pub_vel = node.advertise<geometry_msgs::Twist>("cmd_vel" , 1);
	ros::spin();

}
