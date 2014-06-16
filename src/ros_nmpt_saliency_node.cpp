#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <sstream>
#include "BlockTimer.h"
#include "FastSalience.h"
#include "LQRPointTracker.h"
#include "NMPTUtils.h"
#include <sensor_msgs/image_encodings.h>

#include "geometry_msgs/Point.h"

using namespace std;
using namespace cv; 

Size imSize(320,240); 
BlockTimer bt; 
FastSalience salTracker;
LQRPointTracker salientSpot(2);
vector<double> lqrpt(2,.5); 
Mat im, im2, viz, sal ;
ros::Publisher pub, image_viz_pub;
geometry_msgs::Point pt;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	double saltime, tottime; 
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		im2=cv_ptr->image;
		double ratio = imSize.width * 1. / im2.cols; 
		resize(im2, im, Size(0,0), ratio, ratio, INTER_NEAREST);
		
		viz.create(im.rows, im.cols*2, CV_32FC3);

		bt.blockRestart(0); 
		vector<KeyPoint> pts; 
		salTracker.detect(im, pts); 
		saltime = bt.getCurrTime(0); 

		salTracker.getSalImage(sal); 

		double min, max; 
		Point minloc, maxloc; 
		minMaxLoc(sal, &min, &max, &minloc, &maxloc); 

		lqrpt[0] = maxloc.x*1.0 / sal.cols;  
		lqrpt[1] = maxloc.y*1.0 / sal.rows; 

		salientSpot.setTrackerTarget(lqrpt);
		salientSpot.updateTrackerPosition(); 
		lqrpt = salientSpot.getCurrentPosition();
		
		pt.x = lqrpt[0] * im.cols;
		pt.y = lqrpt[1] * im.rows;
		pt.z = 0;
		pub.publish(pt);
		
		circle(im, Point(pt.x, pt.y), 5, CV_RGB(0,0,255));
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = im;

		image_viz_pub.publish(out_msg.toImageMsg());

		tottime = bt.getCurrTime(1); 
		bt.blockRestart(1);
	}
	catch (...)
	{
		ROS_ERROR("Something went wrong...");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cvStartWindowThread();
	image_transport::ImageTransport it(nh);

	std::string image_topic("/camera/image_raw");

	ros::NodeHandle nhr("~");
	if (nhr.hasParam("image_topic"))
	{
		nhr.getParam("image_topic", image_topic);
	}
	
	image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);
	pub = nh.advertise<geometry_msgs::Point>("/salient_point", 50);
	image_viz_pub = nh.advertise<sensor_msgs::Image>("/salient_point_viz", 1);

	salientSpot.setTrackerTarget(lqrpt);
	bt.blockRestart(1);

	ros::spin();
}
