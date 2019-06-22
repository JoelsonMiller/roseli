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
#include <ecl/threads.hpp>
#include "roseli/GetOdom.h"
#include "roseli/ResetEnc.h"
#define PI 3.14159265

using namespace std;
using namespace ecl;
Mutex mutex;
geometry_msgs::Twist velocity;
std_msgs::Empty reset;
ros::Publisher pub_vel;
ros::Publisher pub_enc;
ros::Publisher pub_state;
ros::Publisher pub_setpoint;
ros::ServiceClient client;
ros::ServiceClient client1;
ros::ServiceClient client2;
void move(float, float);
ros::Duration d(0.01);
float distancia=0;
float angulo=0;
int height;
int width;
float control_data = 0;
double pre_error =0 , integral=0, ant_secs=0;
float sonar_distance = 0;

class Listener
{
public:


//------------------------------------------------------------------------------------------------------
//Função que calcula as compomentes propocionais, derivativas e intergrais
//------------------------------------------------------------------------------------------------------
	
double pid(float pv, double setpoint, double Kp, double Ki, double Kd, double _max, double _min){
	
	double secs =ros::Time::now().toSec();
	double dt = secs - ant_secs;
	
	double error = setpoint - pv;
	double Pout = Kp*error;
	//proportional term
	integral +=error*dt;
	double Iout = Ki*integral;
	
	//integral term
	double derivative = (error - pre_error)/dt;
	double Dout = Kd*derivative;
	
	//derivate term
	double data_control = Pout + Iout + Dout;
	
	if( data_control > _max )
        	data_control = _max;
    	else if( data_control < _min )
        	data_control = _min;
	
	pre_error = error;
	return data_control;
}

//==================================================================================================================

void take_sonar_distance(const std_msgs::Float64::ConstPtr& measure_data){

	//Recebe o valor da medida do sensor e guarda em uma variavel global
	sonar_distance = measure_data->data;

}

int call_srv_map(int interest_num, int intersection_type){

	roseli::CreateMap cm;
	cm.request.pose2d.x = std::numeric_limits<double>::infinity();
        cm.request.pose2d.y = std::numeric_limits<double>::infinity();
        cm.request.pose2d.theta = std::numeric_limits<double>::infinity();
	cm.request.intr_pnt_brd = interest_num;
	cm.request.intr_type = intersection_type;
        if (client.call(cm)){
        	cout<<"Intercao e: "<<cm.response.intr_pnt_graph<< endl;
		return cm.response.intr_pnt_graph;
	}
        else{
        	ROS_ERROR("Failed to call service CreateMap");
                exit(0);
        }
}

void odom_move(float mag, int tipo_movimento){

	
	int flag;
	roseli::GetOdom getodom;
	roseli::ResetEnc resetenc;

	client1.call(resetenc);
	usleep(10000);
	client2.call(getodom);
	angulo = getodom.response.dist.theta;
	distancia = getodom.response.dist.x;

        if( mag > 0)
                flag = 1;
        else
                flag = -1;

        while(1){
                if( (angulo != 0) && (distancia != 0) ){
                        client1.call(resetenc);
			cout<<"Resetando"<<endl;
                        client2.call(getodom);
        		usleep(10000);
        		angulo = getodom.response.dist.theta;
        		distancia = getodom.response.dist.x;
                }
                else
                     break;
                usleep(10000);
        }
        //Continua o programa apenas quando as variaveis angulo e distanica
        // forem resetadas
        if(tipo_movimento == 0){ // 0 == distância e 1 == angulo
                while(abs(distancia)<=abs(mag)){
                        move(flag*0.07, 0);
			client2.call(getodom);
                        usleep(1000);
                        distancia = getodom.response.dist.x;
                        //cout<<"distancia percorrida até agora: "<<distancia<<" valor a ser alcancado: "<<abs(mag)<<endl;
                }
                cout<<"I reached the goal (distance)"<<endl;
                move(0,0);
		usleep(100000);
        }
        else if (tipo_movimento == 1){
                while(abs(angulo) <= abs(mag)){
                        move(0, flag*0.07);
                        //cout<<"angulo girado até agora: "<<angulo<<endl;
                	client2.call(getodom);
                        usleep(1000);
                        angulo = getodom.response.dist.theta;
		}
                cout<<"I reached the goal (angulo)"<<endl;
                move(0,0);
	}
	while(1){
                if( (angulo != 0) && (distancia != 0) ){
                        client1.call(resetenc);
			cout<<"Resetando"<<endl;
                        client2.call(getodom);
                        usleep(10000);
                        angulo = getodom.response.dist.theta;
                        distancia = getodom.response.dist.x;
                }
                else
                     	break;
		usleep(10000);
        }



}

void points_sub(const roseli::PointVector::ConstPtr& points){

	double rightside =0, rightside_center = 0, rightside_up = 0;
	double leftside = 0, leftside_center = 0, leftside_up = 0;
	double res_center, res_up, median_points_up, median_points_up_up, median_points_center, different_median_points, median_points_down;
	double dist;
	float res;
	float ang, p1, p2, adj, opt, adj_major, opt_major;
	std_msgs::Float64 erro;
        std_msgs::Float64 setpoint;
	ros::Rate rate(0.5);
	ros::Rate later(0.2);
	int type_move;

	if(points->points_center.size()==2){
		//Abordagem utilizada por Daniel para seguir a linha
		/*if(points->points_up.size()!=0){
			median_points_center = (points->points_center[1].x - points->points_center[0].x)/2 ;
			median_points_up = (points->points_up[1].x - points->points_up[0].x)/2 ;
			different_median_points = median_points_up - median_points_center;
		}*/
		leftside = abs(points->points_center[0].x);
		rightside = abs(width - points->points_center[1].x);
		res = leftside-rightside;
	
	}

	//cout<<res<<endl;
	//dist = (points->points_center[1].x-points->points_center[0].x);
//	ROS_INFO("Pontos na imagem %f", dist );

	if((points->points_center.size() == 0)||(sonar_distance <= 30.0)){ //Robô não se desloca
			ROS_INFO("O robô perdeu o caminho ou a há um obstáculo");
			move(0,0);

	}else if((points->points_center.size() == 2)){
		median_points_center = (points->points_center[1].x-points->points_center[0].x);
		
		if((points->points_up_up.size()==4) &&
				(points->points_up.size() == 4) &&
					(points->points_down.size() == 2)){
			
			median_points_up_up = (points->points_up_up[2].x - points->points_up_up[1].x) ;
			median_points_up = (points->points_up[2].x - points->points_up[1].x) ;
			//cout<<"I could came here!"<<endl;
			if(median_points_up_up > median_points_up){
				ROS_INFO("Interseção tipo: Y");
				move(0,0);
				type_move = call_srv_map(1, 0);
				if(type_move==0){
					ROS_INFO("Caminho a seguir: Direita");
					odom_move(10, 0);
					odom_move(30, 1);
				}
				else{
					ROS_INFO("Caminho a seguir: Esquerda");
					odom_move(14, 0);
					odom_move(-30, 1);
				}
			}

		}

		else if((points->points_up_up.size() == 2) &&
				(points->points_up.size() == 2) &&
					(points->points_down_down.size()>=4)&&(points->points_down_down.size()<=6)){
			
			if(points->points_down_down[4].x-points->points_down_down[0].x > width/3){
				if((points->points_up_up[1].x-points->points_up_up[0].x)/2 + points->points_up_up[0].x < width/2){
					ROS_INFO("Interseção tipo: Y da direita");
					move(0,0);
					type_move = call_srv_map(1, 1);
					if(type_move==0){
						ROS_INFO("Caminho a seguir: Esquerda");
						odom_move(10, 0);
						odom_move(-30, 1);
					}
					else{
						ROS_INFO("Caminho a seguir: Direita");
						odom_move(14, 0);
						odom_move(120, 1);
					}
				}
				else if((points->points_up_up[1].x-points->points_up_up[0].x)/2 + points->points_up_up[0].x > width/2){
					ROS_INFO("Interseção tipo: Y da esquerda");
					move(0,0);
					type_move = call_srv_map(1, 1);
					if(type_move==0){
						ROS_INFO("Caminho a seguir: Direita");
						odom_move(10, 0);
						odom_move(30, 1);
					}
					else{
						ROS_INFO("Caminho a seguir: Esquerda");
						odom_move(14, 0);
						odom_move(-120, 1);
					}
				}
			}	

		}
		

		else if((points->points_up_up.size() == 0) &&
				(points->points_down.size() >= 4 )&&
					(points->points_down_down.size() >= 4)&&(points->points_down_down.size()<=6)){
			if(points->points_down_down[0].x < width/4){
					ROS_INFO("Interseção do tipo /");
					move(0,0);

					adj = height/6;
					p1 = points->points_down_down[1].x;
					p2 = points->points_down[1].x;
					opt = abs(p1 - p2);
					ang = atan(opt/adj)*180/PI;
					ROS_INFO("O angulo da curva :%f", ang);
					usleep(10000);

					type_move = call_srv_map(1, 2); //Call the creatinmap e insert a intersection node

					if(type_move==0){
						ROS_INFO("Caminho a seguir: angulo reto");
						odom_move(14, 0);
						odom_move(-80, 1);
					}
					else{
						ROS_INFO("Caminho a seguir: angulo agudo");
						odom_move(14, 0);
						odom_move(-170+ang, 1);
					}
				}
				else{
					ROS_INFO("Interseção do tipo \\");
					move(0,0); // para o robô
					adj = height/6;
					p1 = points->points_down_down[2].x;
					p2 = points->points_down[2].x;
					opt = abs(p1 - p2);
					//opt_major = points->points_center[2].x - points->points_center[1].x;
					ang = atan(opt/adj)*180/PI;
					//adj_major = opt_major/tan(ang*PI/180);
					ROS_INFO("O angulo da curva :%f", ang);
					move(0,0);
					usleep(1000000);

					type_move = call_srv_map(1, 2); //Call the creatinmap e insert a intersection node

					if(type_move==0){
						ROS_INFO("Caminho a seguir: angulo reto");
			                        odom_move(14, 0);
			                        odom_move(80, 1);
			                }
			                else{
						ROS_INFO("Caminho a seguir: angulo agudo");
			                        odom_move(14, 0);
			                        odom_move(170-ang, 1);
			                }

				}	
		}
		else if((median_points_center > width/3)&&(median_points_center < 2*width/3)){
			if((points->points_up.size() == 0) && (points->points_down.size() == 2)&&(points->points_down_down.size()==2)){
				if(res < 0){
					ROS_INFO("Angulo de -90 graus");
					move(0,0);
					usleep(1000000);

					type_move = call_srv_map(0, 3); //Call the creatinmap e insert a intersection node

					odom_move(13, 0);
					odom_move(-85, 1);
					odom_move(-5, 0);
				}
				else{
					ROS_INFO("Angulo de 90 graus");
					move(0, 0);
					usleep(1000000);

					type_move = call_srv_map(0, 3); //Call the creatinmap e insert a intersection node

					odom_move(13, 0);
					odom_move(85, 1);
					odom_move(-5, 0);
				}
			}
			else if((points->points_up.size() == 2)&&(points->points_down.size()==2)){
				if(res < 0){
					move(0, 0);
					ROS_INFO("Interseção tipo: -|");

					type_move = call_srv_map(1, 4); //Call the creatinmap e insert a intersection node
					if(type_move==0){
						odom_move(2, 0);
						ROS_INFO("Segue em frente");					
					}
					else{
						ROS_INFO("Caminho a seguir: Curva");
						odom_move(12, 0);
                                        	odom_move(-80, 1);
						odom_move(-5, 0);
					}
				}
				else{
					move(0,0);
					ROS_INFO("Interseção tipo:|-");

					type_move = call_srv_map(1, 4); //Call the creatinmap e insert a intersection node
					if(type_move == 0){
						ROS_INFO("Segue em frente");
						odom_move(2,0);
					}
					else{
						ROS_INFO("Caminho a seguir: Curva");
                                                odom_move(12, 0);
                                                odom_move(80, 1);
						odom_move(-5, 0);
                                        }

				}
			}

			
		}
		else if(median_points_center > 2*width/3){
			if((points->points_up.size() == 0) && (points->points_down.size() == 2)){
				ROS_INFO("Interseção tipo: T");
				move(0,0);
				usleep(1000000);
				type_move = call_srv_map(1, 5);
				if(!type_move){
					ROS_INFO("Caminho a seguir: Direita");
					odom_move(14, 0);
					odom_move(90, 1);
					odom_move(-5.0, 0);
				}
	else{
					ROS_INFO("Caminho a seguir: Esquerda");
					odom_move(14, 0);
					odom_move(-90, 1);
					odom_move(-5.0, 0);
				}
			}
			else if((points->points_up.size()==2)&&(points->points_down.size()==2)){
				ROS_INFO("Interseção tipo: +");
				move(0,0);
				usleep(1000000);

				type_move = call_srv_map(2, 6);

				if(!type_move){
					ROS_INFO("Caminho a seguir: Segue em frente");
					odom_move(1, 0);
				}

				else if(type_move == 2){
					ROS_INFO("Caminho a seguir: Direita");
					odom_move(13, 0);
					odom_move(80, 1);
					odom_move(-5.0, 0);
				}
				else{
					ROS_INFO("Caminho a seguir: Esquerda");
					odom_move(13, 0);
					odom_move(-80, 1);
					odom_move(-5.0, 0);
				}
			}
		}
		
			
		else{
			//Utilização de um PID para o controle de movimento do RoSeLi
				control_data = pid(-res, 0, 0.003, 0, 0.003, 0.3, -0.3); //Funcao teste a PID do ROS
				/*if(ros::param::has("/controller/Kp")){
					if(different_median_points > width/5){
						ros::param::set("/controller/Kp", 0.01);
						ros::param::set("/controller/Kp", 0.01);
						usleep(1000);				
					}
					else{
						ros::param::set("/controller/Kp", 0.005);
						ros::param::set("/controller/Kp", 0.005);
						usleep(1000);					
					}

				}	*/			
				erro.data = -res;
				pub_state.publish(erro);

				setpoint.data = 0;
                        	pub_setpoint.publish(setpoint);
                        	
                        	move(0.15, control_data);
                        	control_data = 0;
		}

	}

	else if((points->points_up_up.size() ==0) &&
			(points->points_down.size() >= 6)&&
				points->points_center.size() == 2){
		if(points->points_center[1].x - points->points_center[0].x < width/3){			
			ROS_INFO("Interseção do tipo: Seta");
			move(0,0);
			usleep(2000000);
			type_move = call_srv_map(1, 7);
				if(!type_move){
					odom_move(14, 0);
					odom_move(-135, 1);

				}else{
					odom_move(14, 0);
					odom_move(135, 1);
					
				}
		}
				
	}


//-----------------------------------------------------------------------
//====== Parte que resolve problema do robô parar quando encontra o padrão de existencia de 4 pontos na linha de cima, do meio e embaixo

	else if((points->points_center.size() == 4)){
		move(0.07, 0);

	}
//-----------------------------------------------------------------------

}	

};

void move(float x, float z){

	ros::Rate rate(100);
	velocity.linear.x =  x;
	velocity.angular.z = z;
	pub_vel.publish(velocity);
}

int main(int argc, char** argv){

	ros::init(argc, argv, "motor_move");
	ros::NodeHandle node;
	pub_vel = node.advertise<geometry_msgs::Twist>("cmd_vel" , 1);
	pub_state = node.advertise<std_msgs::Float64>("/state", 1);
	pub_setpoint = node.advertise<std_msgs::Float64>("/setpoint", 1);

	Listener l;
	ros::Subscriber sub_points=node.subscribe("line/points", 1, &Listener::points_sub, &l);
	ros::Subscriber sub_control = node.subscribe("/sonar_distance", 1, &Listener::take_sonar_distance, &l);

	node.param("/raspicam_node/width", width, 250);
	node.param("/raspicam_node/height", height, 350);
	client = node.serviceClient<roseli::CreateMap>("/pose2D");
	client1 = node.serviceClient<roseli::ResetEnc>("/reset_enc_server");
	client2 = node.serviceClient<roseli::GetOdom>("/odom_server");
	ros::AsyncSpinner s(2);
	s.start();

	ros::Rate r(100);
	ros::spinOnce();
	while (ros::ok()){
		r.sleep();
	}
	return 0;
}

