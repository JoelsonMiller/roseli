#include "ImageConverter.h"
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
void locatepoints(const cv_bridge::CvImagePtr&);

static const std::string OPENCV_WINDOW = "Image window";

 	 ImageConverter::ImageConverter()
		: imageTransport(nh)
  	{
  		imageSubscriber = imageTransport.subscribe("/raspicam_node/image", 1,&ImageConverter::imageCallback, this);
   		// image_pub_ = it_.advertise("/image_converter/output_video", 1);
    		namedWindow(OPENCV_WINDOW);
  	}

  	ImageConverter::~ImageConverter()
  	{
   	destroyWindow(OPENCV_WINDOW);
 	}

  	void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  	{
    		cv_bridge::CvImagePtr img;
    		try
		{
			//ROS_INFO("Trying to convert");
      			img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    			//ROS_INFO("Converted");
		}
    		catch (cv_bridge::Exception& e)
    		{
			ROS_INFO("Not Converter");
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}
	locatepoints(img);
    	// Update GUI Window
    	//cv::imshow(OPENCV_WINDOW, img->image);
   	//	cv::waitKey(3);
  }

void locatepoints(const cv_bridge::CvImagePtr& cv_ptr ){
	//local variable declaration
	//vector<Point> contours;
	vector< vector<Point> > imgContours;
	Mat imgThresholder;
	Mat imgGrayScaled;
	//vector<Vec4i> hierarchy;
	vector<Vec3b> buf;
	vector<int> position;

	//threshold for desired collor
	cvtColor(cv_ptr->image, imgGrayScaled, CV_RGB2GRAY);
	threshold( imgGrayScaled, imgThresholder, 127, 255, 1);	
	//if debug is true, show the desired color cropped from the others
	
	//find the contours in the image
	findContours(imgThresholder, imgContours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	//variable to receive the approximated contours
	//cout<<"contour size   "<<imgContours.size()<<endl;
	vector < vector<Point> > imgContourspoly( imgContours.size() );
	approxPolyDP(Mat(imgContours[0]), imgContourspoly[0], 3,true);
	
	//creates a Mat of zeros to recieve the contours drawing
	Mat drawingContours = Mat::zeros(imgThresholder.size(), CV_8UC3);

	//draw found contour in the created image
	drawContours(drawingContours, imgContourspoly,-1, Scalar(255,0,0),1);
	
	//iterate through a line in the middle of the screen to find its values
	LineIterator it(drawingContours, Point(0,480/2),Point(640,480/2),8);
	
	//variable to recieve found points coordinates	
	vector<Point> points(it.count);

	for(int index = 0; index<it.count; index++){
		buf.push_back( Vec3b(*it) );
		if(buf[index].val[0] != 0){
			points[index]=it.pos();
		}
		it++;
	}
	for(int index = 0; index<it.count; index++){
		if(points[index].x !=0){
			circle(drawingContours, points[index], 10, Scalar(0,0,255),1,8);
			position.push_back(index);
		}
	}
	//imshow(OPENCV_WINDOW, imgThresholder);
	//waitKey('c');
	//imshow("Contours",drawingContours);
	//waitKey(1);
	//imshow("CONTOURS AND FOUND POINTS", drawingContours);
	//waitKey(5);

	double rightside = 0;
	double leftside = 0;
	double res;

	if(!position.empty()){
		//cout<<"here"<<endl;
		leftside = abs(0 - points[position[0]].x);
		rightside = abs(640 - points[position[1]].x);
		res = leftside-rightside;
	}
	
	cout<<res<<endl;
}




