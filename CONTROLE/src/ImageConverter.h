
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <roseli/param_lineConfig.h>
#include <dynamic_reconfigure/server.h>

class ImageConverter{
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport imageTransport;
	image_transport::Subscriber imageSubscriber;
	image_transport::Publisher imagePublisher;
	ros::Publisher pub_vel;
	ros::Publisher pub_points;
	dynamic_reconfigure::Server<roseli::param_lineConfig> server;
        dynamic_reconfigure::Server<roseli::param_lineConfig>::CallbackType f;
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void callback_reconfigure(roseli::param_lineConfig &config, uint32_t level);

public:
	ImageConverter();
	virtual~ImageConverter();
};
