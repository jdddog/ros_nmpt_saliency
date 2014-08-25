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
#include <limits>
#include <std_msgs/Header.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
Size im_size(320,240);
FastSalience salTracker;
LQRPointTracker salientSpot(2);
vector<double> lqrpt(2,.5);
Mat image, resized_image, sal;
ros::Publisher point_pub, image_viz_pub;
geometry_msgs::Point point;
int radius;
bool visualize;


float clamp(float value, float min, float max)
{
    return value < min ? min : (value > max ? max : value);
}

void update_visualization(Mat image, std_msgs::Header header, int x, int y)
{
    circle(image, Point(x, y), 5, CV_RGB(0,0,255));
    cv_bridge::CvImage viz_msg;
    viz_msg.header = header;
    viz_msg.encoding = sensor_msgs::image_encodings::BGR8;
    viz_msg.image = image;
    image_viz_pub.publish(viz_msg.toImageMsg());
}

geometry_msgs::Point get_average_point(PointCloud cloud, int x, int y, int radius)
{
    geometry_msgs::Point average_point;
    pcl::PointXYZRGB point;

    int width = cloud.width;
    int height = cloud.height;
    int x_start = clamp(x - radius, 0, width);
    int x_end = clamp(x + radius, 0, height);
    int y_start = clamp(y - radius, 0, height);
    int y_end = clamp(y + radius, 0, height);
    int cells_visited = 0;

    for(int i = x_start; i < x_end; i++)
    {
        for(int j = y_start; j < y_end; j++)
        {
            point = cloud.at(i, j);
            average_point.x += point.x;
            average_point.y += point.y;
            average_point.z += point.z;
            cells_visited++;tf::TransformBroadcaster br;
            
        }

    }

    average_point.x = average_point.x / cells_visited;
    average_point.y = average_point.y / cells_visited;
    average_point.z = average_point.z / cells_visited;

    return average_point;
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{

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

    try
    {
        // Resize image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
		image = cv_ptr->image;

		double ratio = im_size.width * 1. / image.cols;
		resize(image, resized_image, Size(0,0), ratio, ratio, INTER_NEAREST);

		vector<KeyPoint> pts;
		salTracker.detect(resized_image, pts);
		salTracker.getSalImage(sal);

		double min, max;
		Point minloc, maxloc;
		minMaxLoc(sal, &min, &max, &minloc, &maxloc);

		lqrpt[0] = maxloc.x*1.0 / sal.cols;
		lqrpt[1] = maxloc.y*1.0 / sal.rows;

		salientSpot.setTrackerTarget(lqrpt);
		salientSpot.updateTrackerPosition();
		lqrpt = salientSpot.getCurrentPosition();

		int im_x = lqrpt[0] * resized_image.cols;
		int im_y = lqrpt[1] * resized_image.rows;

        // Get 3d point
        //geometry_msgs::Point world_point = get_average_point(cloud, im_x, im_y, radius);
        ROS_INFO("WIDTH: %d, HEIGHT: %d", image.cols, image.rows);
        int b_x = im_x * (image.cols / im_size.width);
        int b_y = im_y * (image.rows / im_size.height);
        pcl::PointXYZRGB world_point = cloud.at(b_x, b_y);
        ROS_INFO("scale: %f, im_x: %d, im_y: %d, b_x: %d, b_y %d, x: %f, y: %f, z: %f", ratio, im_x, im_y, b_x, b_y, world_point.x, world_point.y, world_point.z);

        /*// Publish tf
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(point.x, point.y, point.z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), msg->header.frame_id, "salient_point"));

        // Publish point
        geometry_msgs::PointStamped point_stamped;
        point_stamped.header.stamp = msg->header.stamp;
        point_stamped.header.frame_id = msg->header.frame_id;
        point_stamped.point = point;
		point_pub.publish(point_stamped);*/

        // Visualize
		if(visualize)
		{
            update_visualization(resized_image, msg->header, im_x, im_y);
		}
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str ());
    }
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		image = cv_ptr->image;

		double ratio = im_size.width * 1. / image.cols;
		resize(image, resized_image, Size(0,0), ratio, ratio, INTER_NEAREST);

		vector<KeyPoint> pts;
		salTracker.detect(resized_image, pts);
		salTracker.getSalImage(sal);

		double min, max;
		Point minloc, maxloc;
		minMaxLoc(sal, &min, &max, &minloc, &maxloc);

		lqrpt[0] = maxloc.x*1.0 / sal.cols;
		lqrpt[1] = maxloc.y*1.0 / sal.rows;

		salientSpot.setTrackerTarget(lqrpt);
		salientSpot.updateTrackerPosition();
		lqrpt = salientSpot.getCurrentPosition();

        geometry_msgs::PointStamped point_stamped;
        point_stamped.header.stamp = msg->header.stamp;
        point_stamped.header.frame_id = msg->header.frame_id;
		point_stamped.point.x = lqrpt[0] * resized_image.cols;
		point_stamped.point.y = lqrpt[1] * resized_image.rows;
		point_stamped.point.z = numeric_limits<float>::max();
		point_pub.publish(point_stamped);
		
		ROS_INFO("x: %d, y: %d", (int)point_stamped.point.x, (int)point_stamped.point.y);

        if(visualize)
        {
		    update_visualization(resized_image, msg->header, point_stamped.point.x, point_stamped.point.y);
		}
	}
	catch (Exception e)
	{
		ROS_ERROR("Image callback: %s", e.what());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	ros::NodeHandle nhr("~");
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub_image;
	ros::Subscriber sub_pcl;



	std::string image_topic;
	std::string point_cloud_topic;
	bool use_depth;

	nhr.param<string>("image_topic", image_topic, "/usb_cam/image_raw");
	nhr.param<string>("point_cloud_topic", point_cloud_topic, "");
	nhr.param<bool>("visualize", visualize, true);
	nhr.param<bool>("use_depth", use_depth, false);

	ROS_INFO("Image topic: %s", image_topic.c_str());
	ROS_INFO("Point cloud topic: %s", point_cloud_topic.c_str());
	ROS_INFO("Use depth: %s", use_depth ? "true" : "false");
	ROS_INFO("Visualisation: %s", visualize ? "true" : "false");
	ROS_INFO("Radius: %d", radius);

	if(!use_depth)
	{
        sub_image = it.subscribe(image_topic, 1, image_callback);
	}
	else
	{
        sub_pcl = nh.subscribe(point_cloud_topic, 1, point_cloud_callback);
	}

	if(visualize)
	{
        image_viz_pub = nh.advertise<sensor_msgs::Image>("/salient_point_viz", 1);
	}

	point_pub = nh.advertise<geometry_msgs::PointStamped>("/salient_point", 50);
	ros::spin();
}
