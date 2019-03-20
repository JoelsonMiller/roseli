
#include "ImageConverter.h"
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32MultiArray.h"
#include "roseli/PointVector.h"
#include "geometry_msgs/Point.h"

using namespace cv;
using namespace std;
int height;
int width;
void locatepoints(const cv_bridge::CvImagePtr , image_transport::Publisher, ros::Publisher);

static const std::string OPENCV_WINDOW = "Image window";

  ImageConverter::ImageConverter()
		: imageTransport(nh){
  		imageSubscriber = imageTransport.subscribe("/raspicam_node/image", 1, &ImageConverter::imageCallback, this);
		imagePublisher = imageTransport.advertise("cropTag", 1);
    		pub_points = nh.advertise<roseli::PointVector>("line/points", 1);
		//namedWindow(OPENCV_WINDOW);
		nh.getParam("/raspicam_node/height", height);
		nh.getParam("/raspicam_node/width", width);
		}

  	ImageConverter::~ImageConverter(){
 	  	//destroyWindow(OPENCV_WINDOW);
 	}

void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  	{
    		cv_bridge::CvImagePtr img;
    		try
		{
		//	ROS_INFO("Trying to convert");
      			img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		//	ROS_INFO("Converted");
		}
    		catch (cv_bridge::Exception& e)
    		{
			ROS_INFO("Not Converter");
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}

	//cv::imshow(OPENCV_WINDOW, img->image);
        //cv::waitKey(3);
	locatepoints(img, imagePublisher, pub_points);
  }

void locatepoints(const cv_bridge::CvImagePtr img, image_transport::Publisher imagePublisher, ros::Publisher pub_points ){

	vector< vector<Point> > imgContours, imgContoursTag;
	Mat imgThresholder, imgThresholderTag, imgGrayScaled, image_HSV, imgHSV, erode_img, element1, element2;
	vector<Vec3b> buf0;
	vector<Vec3b> buf1;
	vector<Vec3b> buf2;
	cvtColor(img->image, imgGrayScaled, CV_RGB2GRAY);
	//cvtColor(img->image, imgHSV, CV_BGR2HSV);

	threshold( imgGrayScaled, imgThresholder, 10, 255, 1);
	//inRange(imgHSV, Scalar(0, 0, 0), Scalar(110, 255, 30), imgThresholder);

	//cout<<"Erro Threshold"<<endl;
	//find the contours in the image
	findContours(imgThresholder, imgContours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	//cout<<"Erro FindContours"<<endl;
	//variable to receive the approximated contours
	//cout<<"contour size   "<<imgContours.size()<<endl;
	//imshow("Teste_threshold", imgThresholder);
        //waitKey(5);
	//imshow("Teste_GrayScaled", imgGrayScaled);
	//waitKey(3);

	Mat drawingContours = Mat::zeros(imgThresholder.size(), CV_8UC3);
	if(imgContours.size()!=0){
		vector < vector<Point> > imgContourspoly( imgContours.size() );
		//approxPolyDP(Mat(imgContours[0]), imgContourspoly[0], 3,true);
		//cout<<"Erro PolyDp"<<endl;
		//creates a Mat of zeros to recieve the contours drawing
		//Mat drawingContours = Mat::zeros(imgThresholder.size(), CV_8UC3);
		//vector< vector<Point> > contourspoly_HSV( contours_HSV.size() );
		for (size_t i=0; i<imgContours.size(); i++){
                	approxPolyDP(Mat(imgContours[i]), imgContourspoly[i], 3,true);
        	}
		//draw found contour in the created image
		drawContours(drawingContours, imgContourspoly,-1, Scalar(255,0,0),1);
		//cout<<"erro no draw"<<endl;
		//iterate through a line in the middle of the screen to find its values
		//}
		LineIterator it0(drawingContours, Point(0, height/2), Point(width, height/2), 8);
		LineIterator it1(drawingContours, Point(0, height/4), Point(width, height/4), 8);
		LineIterator it2(drawingContours, Point(0, 3*height/4), Point(width, 3*height/4), 8);
		//cout<<"erro LineIterator"<<endl;
		//variable to recieve found points coordinates
		vector<Point> points0(it0.count);
		vector<Point> points1(it1.count);
		vector<Point> points2(it2.count);
		vector<Point> p0;
		vector<Point> p1;
		vector<Point> p2;
		for(int index = 0; index<it0.count; index++){
			buf0.push_back( Vec3b(*it0) );
			if(buf0[index].val[0] != 0){
				points0[index]=it0.pos();
			}
			it0++;
		}

		for(int index = 0; index<it1.count; index++){
                        buf1.push_back( Vec3b(*it1) );
                        if(buf1[index].val[0] != 0){
                                points1[index]=it1.pos();
                        }
                        it1++;
                }

		for(int index = 0; index<it2.count; index++){
                        buf2.push_back( Vec3b(*it2) );
                        if(buf2[index].val[0] != 0){
                                points2[index]=it2.pos();
                        }
                        it2++;
                }

		for(int index = 0; index<it0.count; index++){
			if(points0[index].x !=0){
				circle(drawingContours, points0[index], 10, Scalar(0,0,255),1,8);
				//position.push_back(index);
				p0.push_back(points0[index]);
			}
		}

		for(int index = 0; index<it1.count; index++){
                        if(points1[index].x !=0){
                                circle(drawingContours, points1[index], 10, Scalar(0,0,255),1,8);
                                //position.push_back(index);
                                p1.push_back(points1[index]);
                        }
                }

		for(int index = 0; index<it2.count; index++){
                        if(points2[index].x !=0){
                                circle(drawingContours, points2[index], 10, Scalar(0,255,0),1,8);
                                //position.push_back(index);
                                p2.push_back(points2[index]);
                        }
                }


		if(p0.size()!=0){
			roseli::PointVector points;
			points.points_center.clear();
			points.points_up.clear();
			points.points_down.clear();
			//cout<<"2"<<endl;
			int i0=0;
			for(vector<Point>::iterator it0 = p0.begin(); it0 != p0.end(); ++it0){
				geometry_msgs::Point point_aux0;
				point_aux0.x=(*it0).x;
				point_aux0.y=(*it0).y;
				point_aux0.z=0;
				points.points_center.push_back(point_aux0);
			i0++;
			}

			int i1=0;
                        for(vector<Point>::iterator it1 = p1.begin(); it1 != p1.end(); ++it1){
                                geometry_msgs::Point point_aux1;
                                point_aux1.x=(*it1).x;
                                point_aux1.y=(*it1).y;
                                point_aux1.z=0;
                                points.points_up.push_back(point_aux1);
                        i1++;
                        }

			int i2=0;
                        for(vector<Point>::iterator it2 = p2.begin(); it2 != p2.end(); ++it2){
                                geometry_msgs::Point point_aux2;
                                point_aux2.x=(*it2).x;
                                point_aux2.y=(*it2).y;
                                point_aux2.z=0;
                                points.points_down.push_back(point_aux2);
                        i1++;
                        }

			points.flag=i0;

			pub_points.publish(points);
		}
		//ros::spin();
		//loop_rate.sleep();

		//imshow(OPENCV_WINDOW, imgThresholder);
		//waitKey('c');
		imshow("Contours",drawingContours);
		waitKey(1);
		//imshow("CONTOURS AND FOUND POINTS", drawingContours);
		//waitKey('c');
			if(p0.size()==0){
				ros::Rate rate(0.4);
				//cvtColor(img->image, image_HSV, CV_BGR2HSV);
                        	inRange(img->image, Scalar(0,0,100), Scalar(35,20,220), imgThresholderTag);
				//threshold( imgGrayScaled, imgThresholderTag, 100, 255, 4);
				findContours(imgThresholderTag, imgContoursTag, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
				//Mat erode;
				int erosion_type = MORPH_RECT;
				int erosion_size = 1, dilate_size = 4;
				element1 = getStructuringElement(erosion_type, 
						Size(2*erosion_size+1, 2*erosion_size+1), 
						Point(erosion_size, erosion_size));
				erode(imgThresholderTag, erode_img, element1);
				element2 = getStructuringElement(erosion_type, 
                                                Size(2*dilate_size+1, 2*dilate_size+1), 
                                                Point(dilate_size, dilate_size));
				dilate(erode_img, imgThresholderTag, element2);
				imshow( "ThresholderTag", imgThresholderTag);
	                        waitKey('c');
				//ROS_INFO("Problem about finding contours");
				if(imgContoursTag.size()==0){
					ROS_INFO("Tag Not Found!");
				}
				else{
					Rect boundRect;
				 	vector < vector<Point> > imgContoursTagPoly(imgContoursTag.size() );
					double maxArea = 0.0;
					cout<<"Problem about finding contours"<<endl;
					for(int i =0; i<imgContoursTag.size(); i++){
						double area = contourArea(imgContoursTag[i]);
						if(area > maxArea){
							maxArea = area;
							approxPolyDP(Mat(imgContoursTag[i]), imgContoursTagPoly[i], 3, true);
							boundRect = boundingRect(Mat(imgContoursTagPoly[i]));
						}
					}
					//ROS_INFO("Problem is here");
					Mat drawingContoursTag = Mat::zeros(imgThresholderTag.size(), CV_8UC3);
                			for (size_t i=0; i<imgContoursTag.size(); i++){
                        			approxPolyDP(Mat(imgContoursTag[i]), imgContoursTagPoly[i], 3,true);
                			}
					drawContours(drawingContoursTag, imgContoursTagPoly, -1 , Scalar(255,0,0),1);
					boundRect.height = boundRect.height/2;
					Mat cropImage = img->image(boundRect);
					//imshow( "Contours",drawingContoursTag);
					//waitKey('v');
					imshow("Imagem cortada", cropImage);
					waitKey('r');
					img->image = cropImage;
					imagePublisher.publish(img->toImageMsg());
					rate.sleep();
					//destroyWindow("Imagem Cortada");
				 }
			}
		}
		else{
			//cout<<"NO FOUND CONTOURS"<<endl;
		}
}

