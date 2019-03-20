
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <vector>

class ImageConverter{
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport imageTransport;
	image_transport::Subscriber imageSubscriber;
	image_transport::Publisher imagePublisher;
	ros::Publisher pub_vel;
	ros::Publisher pub_points;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
	ImageConverter();
	virtual~ImageConverter();
};
