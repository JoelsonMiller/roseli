
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "roseli/PointVector.h"
#include "geometry_msgs/Twist.h"

using namespace std;
geometry_msgs::Twist velocity;
ros::Publisher pub_vel;
void move(float, float);

void points_sub(const roseli::PointVector::ConstPtr& points){

	double rightside = 0;
	double leftside = 0;
	double res;
	double dist;
	ros::Rate rate(0.5);
	ros::Rate later(0.2);

	if(points->points_center.size()!=0){

		leftside = abs(0 - points->points_center[0].x);
		rightside = abs(640 - points->points_center[1].x);
		res = leftside-rightside;
	}

//	cout<<res<<endl;
	dist = (points->points_center[1].x-points->points_center[0].x);
//	ROS_INFO("Pontos na imagem %f", dist );

	if((points->points_center.size()  == 1)||(points->points_center.size() == 0)){
			move(0,0);

	}else if(points->points_center.size() == 2){
		 if((points->points_center[1].x-points->points_center[0].x)>200){
			if(points->points_up.size() == 0){
				if(res < 0){
					cout<<"Angulo de -90 graus"<<endl;
					move(0,0);
				}
				else{
					cout<<"Angulo de 90 graus"<<endl;
					move(0,0);
				}
			}
			else if(points->points_up.size() == 2){
				if(res<0){
					cout<<"Interseção tipo: -|"<<endl;
					move(0,0);
				}
				else{
					cout<<"Interseção tipo:|-"<<endl;
					move(0,0);
				}
			}
			else if(points->points_up.size() == 4){
				cout<<"Interseção tipo: Y"<<endl;
				move(0,0);
			}
		}
		else{
			move(0.2, -res/500);
		}

	}else if(points->points_center.size()==4){
			if(points->points_center[0].x < 50){
				cout<<"Interseção do tipo /"<<endl;
				//move(0.2, 0);
				//usleep(2000000);
				move(0, 0);
				//usleep(4000000);
			}
			else{
				cout<<"I am here"<<endl;
				cout<<"Interseção do tipo / (invertida)"<<endl;
				move(0,0);
				//usleep(5000000);
				//move(0,0.2);
				//usleep(5000000);
			}
			//ROS_INFO("Pontos na imagem %lu", points->points_center.size() );
			//move(0.25, 0);
			//rate.sleep();
			//move(0, -0.85);
			//rate.sleep();
		//	move(0.1 , 0);
                  //      rate.sleep();
		//	move(0 , 0);
                  //      pause.sleep();
	}
}

void move(float x, float z){

	ros::Rate rate(1000);
	velocity.linear.x =  x;
	velocity.angular.z = z;
	pub_vel.publish(velocity);
	rate.sleep();
}


int main(int argc, char** argv){

	ros::init(argc, argv, "Motor_Move");
	ros::NodeHandle node;
	ros::Subscriber sub_points=node.subscribe("line/points", 1, points_sub);
	pub_vel = node.advertise<geometry_msgs::Twist>("cmd_vel" , 1);
	ros::spin();

}
