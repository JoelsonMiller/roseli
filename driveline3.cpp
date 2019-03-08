#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "roseli/PointVector.h"
#include "geometry_msgs/Twist.h"
#include <boost/thread.hpp>
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Empty.h"
#include "math.h"
#include "std_msgs/Float64.h"
#define PI 3.14159265

using namespace std;
geometry_msgs::Twist velocity;
std_msgs::Empty reset;
ros::Publisher pub_vel;
ros::Publisher pub_enc;
ros::Publisher pid;
void move(float, float);
ros::Duration d(0.01);
float distancia=0;
float angulo=0;
int height;
int width;

class Listener
{
public:

void read_encoder(const geometry_msgs::Pose2D::ConstPtr& msg){
	distancia = msg->x;
	angulo = msg->theta;
	d.sleep();
}

void points_sub(const roseli::PointVector::ConstPtr& points){

	double rightside = 0;
	double leftside = 0;
	double res;
	double dist;
	float ang, down, center, adj, opt;
	std_msgs::Float64 plot;
	ros::Rate rate(0.5);
	ros::Rate later(0.2);

	if(points->points_center.size()!=0){

		leftside = abs(points->points_center[0].x);
		rightside = abs(width - points->points_center[1].x);
		res = leftside-rightside;
	}

	//cout<<res<<endl;
	//dist = (points->points_center[1].x-points->points_center[0].x);
//	ROS_INFO("Pontos na imagem %f", dist );

	if((points->points_center.size()  == 1)||(points->points_center.size() == 0)){
			move(0,0);

	}else if(points->points_center.size() == 2){
		 if((points->points_center[1].x-points->points_center[0].x)>width/3){
			if((points->points_up.size() == 0)&&(points->points_down.size()==2)){
				if(res < 0){
					cout<<"Angulo de -90 graus"<<endl;
					move(0,0);
				}
				else{
					cout<<"Angulo de 90 graus"<<endl;
					move(0,0);
				}
			}
			else if((points->points_up.size() == 2)&&(points->points_down.size()==2)){
				if(res<0){
					cout<<"Interseção tipo: -|"<<endl;
					move(0,0);
				}
				else{
					move(0,0);
					usleep(1000000);
					pub_enc.publish(reset);
					cout<<"Interseção tipo:|-"<<endl;
				}
			}
			else if(points->points_up.size() == 4){
				cout<<"Interseção tipo: Y"<<endl;
				move(0,0);
			}
		}
		else if((points->points_center[1].x-points->points_center[0].x)>2*width/3){
			if((points->points_up.size()==0)&&(points->points_down.size()==2)){
				cout<<"Interseção tipo: T"<<endl;
				move(0,0);
			}
			else if((points->points_up.size()==2)&&(points->points_down.size()==2)){
				cout<<"Interseção tipo: +"<<endl;
				move(0,0);
			}
		}
		else{
				move(0.4, res/325);
				plot.data = res/325;
				pid.publish(plot);
		}

	}else if((points->points_center.size()==4)&&(points->points_down.size()==4)){
			if(points->points_center[0].x < width/5){
				cout<<"Interseção do tipo /"<<endl;
				move(0,0);
			}
			else{
				cout<<"Interseção do tipo / (invertida)"<<endl;
				move(0,0);
				adj = height/4;
				down = points->points_down[2].x;
				center = points->points_center[2].x;
				opt = abs(down - center);
				ang = atan(opt/adj)*180/PI;
				ROS_INFO("O angulo da curva :%f", ang);
				usleep(1000000);
				pub_enc.publish(reset);

				while(1){
					if((angulo != 0)&&(distancia != 0)){
						//cout<<"angulo = "<< angulo << endl;
						//cout<<"distancia = "<< distancia << endl;
					}
					else
						break;
					pub_enc.publish(reset);
					usleep(1000000);
				}
	//Continua o programa apenas quando as variaveis angulo e distanica
	// forem resetadas
				while(distancia < 14){
					move(0.2, 0);
				}

				cout<<"I reached the goal (distance)"<<endl;
				move(0,0);
                                pub_enc.publish(reset);

				while(1){
                                        if((angulo!=0)&&(distancia!=0)){
                                                //cout<<"angulo = "<< angulo << endl;
                                                //cout<<"distancia = "<< distancia << endl;
					}
                                        else
						break;
					pub_enc.publish(reset);
					usleep(1000000);
                                }

        //Continua o programa apenas quando as variaveis angulo e distanica
        // forem resetadas

                                while(angulo < 75){
                                        move(0, 0.2);
                                }

				cout<<"I reached the goal (angulo)"<<endl;
				usleep(1000);
				move(0,0);
			}
	}
}

};

void move(float x, float z){

	ros::Rate rate(100);
	velocity.linear.x =  x;
	velocity.angular.z = z;
	pub_vel.publish(velocity);
	rate.sleep();
}


int main(int argc, char** argv){

	ros::init(argc, argv, "motor_move");
	ros::NodeHandle node;
	pub_vel = node.advertise<geometry_msgs::Twist>("cmd_vel" , 1);
	pub_enc = node.advertise<std_msgs::Empty>("/reset_enc", 1);
	pid = node.advertise<std_msgs::Float64>("/pid", 1);
	Listener l;
	ros::Subscriber sub_points=node.subscribe("line/points", 1, &Listener::points_sub, &l);
	ros::Subscriber sub_enc=node.subscribe("/odom", 1, &Listener::read_encoder, &l);
	node.getParam("/raspicam_node/width", width);
	node.getParam("/raspicam_node/height", height);
	ros::AsyncSpinner s(2);
	s.start();

	ros::Rate r(100);
	while (ros::ok()){
		r.sleep();
	}
	return 0;
}
