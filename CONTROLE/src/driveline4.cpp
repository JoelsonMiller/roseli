#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "roseli/PointVector.h"
#include "roseli/CreateMap.h"
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
ros::Publisher pub_state;
ros::Publisher pub_setpoint;
void move(float, float);
ros::Duration d(0.01);
float distancia=0;
float angulo=0;
int height;
int width;
float control_data = 0;

class Listener
{
public:

void receive_data_control(const std_msgs::Float64::ConstPtr& controller){
	control_data = controller->data;
}


void read_encoder(const geometry_msgs::Pose2D::ConstPtr& msg){
	distancia = msg->x;
	angulo = msg->theta;
	d.sleep();
}

void odom_move(float mag, int tipo_movimento){

	int flag;
	pub_enc.publish(reset);

	if( mag > 0)
		flag = 1;
	else
		flag = -1;
	
        while(1){
        	if( (angulo != 0) && (distancia != 0) ){
                	cout<<"Resetando"<<endl;
                       	pub_enc.publish(reset);
                }
                else
                     break;
                usleep(1000000);
        }
        //Continua o programa apenas quando as variaveis angulo e distanica
        // forem resetadas
	if(tipo_movimento == 0){ // 0 == distância e 1 == angulo
                while(distancia <= abs(mag))
                	move(flag*0.07, 0);

                cout<<"I reached the goal (distance)"<<endl;
                move(0,0);
	}
	else if (tipo_movimento == 1){
		while(angulo <= abs(mag))
                        move(0, flag*0.07);

                cout<<"I reached the goal (angulo)"<<endl;
                move(0,0);
	}
}

void points_sub(const roseli::PointVector::ConstPtr& points){

	double rightside = 0;
	double leftside = 0;
	double res;
	double dist;
	float ang, down, center, adj, opt, adj_major, opt_major;
	std_msgs::Float64 erro;
        std_msgs::Float64 setpoint;
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

	if((points->points_center.size()  == 1)||(points->points_center.size() == 0)){ //Robô não se desloca
			move(0,0);

	}else if(points->points_center.size() == 2){
		 if((points->points_center[1].x-points->points_center[0].x) > width/3){
			if((points->points_up.size() == 0) && (points->points_down.size() == 2)){
				if(res < 0){
					cout<<"Angulo de -90 graus"<<endl;
					odom_move(-90, 1);
				}
				else{
					cout<<"Angulo de 90 graus"<<endl;
					odom_move(90, 1);
				}
			}
			else if((points->points_up.size() == 2)&&(points->points_down.size()==2)){
				if(res < 0){
					move(0, 0);
					cout<<"Interseção tipo: -|"<<endl;
				}
				else{
					move(0,0);
					cout<<"Interseção tipo:|-"<<endl;
				}
			}
			else if(points->points_up.size() == 4){
				cout<<"Interseção tipo: Y"<<endl;
				move(0,0);
			}
		}
		else if((points->points_center[1].x - points->points_center[0].x) > 2*width/3){
			if((points->points_up.size() == 0) && (points->points_down.size() == 2)){
				cout<<"Interseção tipo: T"<<endl;
				odom_move(14, 0);
				odom_move(90, 1)
			}
			else if((points->points_up.size()==2)&&(points->points_down.size()==2)){
				cout<<"Interseção tipo: +"<<endl;
				odom_move(5, 0);
			}
		}
		else{
				//move(0.4, res/350);
				//plot.data = res/350;
				//pid.publish(plot);
			//Utilizaçãode um PID para o controle de movimento do RoSeLi
				erro.data = -res;
				setpoint.data = 0;
                        	pub_setpoint.publish(setpoint);
                        	usleep(1000);
                        	pub_state.publish(erro);
                        	usleep(1000);
                        	move(0.33, control_data);
                        	control_data = 0;
		}

	}else if(((points->points_center.size()==4)||(points->points_center.size()==5))&&((points->points_down.size()==5)||(points->points_down.size()==4))){
			if(points->points_center[0].x < width/5){
				cout<<"Interseção do tipo /"<<endl;
				move(0,0);
				adj = height/4;
				down = points->points_down[1].x;
				center = points->points_center[1].x;
				opt = abs(down - center);
				//opt_major = points->points_center[2].x - points->points_center[1].x;
				ang = atan(opt/adj)*180/PI;
				//adj_major = opt_major/tan(ang*PI/180);
				ROS_INFO("O angulo da curva :%f e o valor de distancia: %f", ang, adj_major);
				usleep(1000000);
				move(0,0);
				odom_move(14, 0);
				odom_move(-80, 1);
			}
			else{
				cout<<"Interseção do tipo / (invertida)"<<endl;
				move(0,0); // para o robô
				adj = height/4;
				down = points->points_down[2].x;
				center = points->points_center[2].x;
				opt = abs(down - center);
				//opt_major = points->points_center[2].x - points->points_center[1].x;
				ang = atan(opt/adj)*180/PI;
				//adj_major = opt_major/tan(ang*PI/180);
				ROS_INFO("O angulo da curva :%f e o valor de distancia: %f", ang, adj_major);
				usleep(1000000);
				move(0,0);
				odom_move(14, 0);
				odom_move(80, 1);
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
	pub_state = node.advertise<std_msgs::Float64>("state", 1);
	pub_setpoint = node.advertise<std_msgs::Float64>("setpoint", 1);
	Listener l;
	ros::Subscriber sub_points=node.subscribe("line/points", 1, &Listener::points_sub, &l);
	ros::Subscriber sub_enc=node.subscribe("/odom", 1, &Listener::read_encoder, &l);
	ros::Subscriber sub_control = node.subscribe("/control_effort", 1, &Listener::receive_data_control, &l);
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
