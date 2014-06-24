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

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

using namespace std;
using namespace cv; 

Size imSize(320,240); 
BlockTimer bt; 
FastSalience salTracker;
LQRPointTracker salientSpot(2);
vector<double> lqrpt(2,.5); 
Mat im, im2, viz, sal ;
ros::Publisher pub, image_viz_pub, pointcloud_viz_pub;
geometry_msgs::Point pt;



// Good name!
void pointcloudCallback (const sensor_msgs::PointCloud2ConstPtr & msg)
{
    //printf("Lol being called rofl.");

    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

    /* convert cloud to PCL & PCLPointCloud2 */
    PointCloud cloud;
    pcl::PCLPointCloud2 cloud_2;
    pcl_conversions::toPCL(*msg, cloud_2);
    pcl::fromPCLPointCloud2(cloud_2, cloud);


    /* get an OpenCV image from the cloud */
    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(cloud_2, pcl_image);
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str ());
    }

    //pointcloud_viz_pub.publish(image_msg);

	double saltime, tottime; 
//	cv_bridge::CvImagePtr cv_ptr;

	try
	{
        //cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
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
		
		pub.publish(pt);

        int im_x = lqrpt[0] * im.cols;
        int im_y = lqrpt[1] * im.cols;

        pcl::PointXYZRGB goodPoint = cloud.at(im_x, im_y);

        pt.x = goodPoint.x;
        pt.y = goodPoint.y;
        pt.z = goodPoint.z;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(pt.x, pt.y, pt.z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "salient_point"));
		
        circle(im, Point(im_x, im_y), 5, CV_RGB(0,0,255));
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
    std::string pointcloud_topic("/camera/depth_registered/points");

	ros::NodeHandle nhr("~");
	if (nhr.hasParam("image_topic"))
	{
		nhr.getParam("image_topic", image_topic);
	}
	
    //image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);
    ros::Subscriber subPcl = nh.subscribe(pointcloud_topic, 1, pointcloudCallback);

    pub = nh.advertise<geometry_msgs::Point>("/salient_point", 50);
    image_viz_pub = nh.advertise<sensor_msgs::Image>("/salient_point_viz", 1);

    //pointcloud_viz_pub = nh.advertise<sensor_msgs::Image>("/salient_cloud_viz", 1);

    salientSpot.setTrackerTarget(lqrpt);
    bt.blockRestart(1);

	ros::spin();
}
