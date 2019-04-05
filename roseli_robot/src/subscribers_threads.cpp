#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Empty.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system, using
 * an asynchronous Spinner object to receive callbacks in multiple threads at the same time.
 */

ros::Duration d(0.01);
ros::Publisher pub1;
ros::Publisher pub2;

float distancia = 0;
float angulo = 0;

class Listener
{
public:
  void odometry(const geometry_msgs::Pose2D::ConstPtr& msg)
  {
   // ROS_INFO_STREAM("chatter1: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
   	distancia = msg->x;
	angulo = msg->theta;
	d.sleep();
  }
  void driveline(const std_msgs::String::ConstPtr& msg)
  {
    //ROS_INFO_STREAM("chatter2: [" << msg->data << "] [thread=" << boost::this_thread::get_id() << "]");
    	geometry_msgs::Twist velocity;
	std_msgs::Empty reset;
	if(msg->data=="start"){
		pub2.publish(reset);
		usleep(100000);
                while(angulo < 90){
			velocity.linear.x = 0;
                        velocity.angular.z = 0.2;
                        pub1.publish(velocity);
                	usleep(1000);
		}
		ROS_INFO("I reached the goal!");
        }
	else if(msg->data=="finished"){
		//ROS_INFO("Finished MotherFucker!");
		velocity.linear.x = 0.1;
		velocity.angular.z = 0;
                pub1.publish(velocity);
                usleep(10000);
	}
	else{
		velocity.linear.x = 0;
		velocity.angular.z = 0;
                pub1.publish(velocity);
                usleep(10000);
	}
	d.sleep();
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  pub1 = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  pub2 = n.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",1);
  Listener l;
  ros::Subscriber sub1 = n.subscribe("/odom_pose2D", 1, &Listener::odometry, &l);
  ros::Subscriber sub2 = n.subscribe("/chatter", 1, &Listener::driveline, &l);
  //ros::Subscriber sub3 = n.subscribe("chatter", 10, &Listener::chatter3, &l);
  //ros::Subscriber sub4 = n.subscribe("chatter", 10, chatter4);

  /**
   * The AsyncSpinner object allows you to specify a number of threads to use
   * to call callbacks.  If no explicit # is specified, it will use the # of hardware
   * threads available on your system.  Here we explicitly specify 4 threads.
   */
  ros::AsyncSpinner s(2);
  s.start();

  ros::Rate r(5);
  while (ros::ok())
  {
  //  ROS_INFO_STREAM("Main thread [" << boost::this_thread::get_id() << "].");
    r.sleep();
  }

  return 0;
}

