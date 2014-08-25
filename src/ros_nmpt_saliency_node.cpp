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
#include "ros_nmpt_saliency/SalientPoint2D.h"
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace cv;
using namespace ros_nmpt_saliency;

// Saliency tracking
Size im_size(320,240);
FastSalience salTracker;
LQRPointTracker salientSpot(2);
vector<double> lqrpt(2,.5);
Mat image, resized_image, sal;

// ROS
int radius;
bool visualize;
ros::Publisher sal_2d_pub, sal_3d_pub, image_viz_pub;
ros::Subscriber sub_pcl, sub_cam_info;
geometry_msgs::Point point;

// Depth
sensor_msgs::PointCloud2 point_cloud_msg;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
sensor_msgs::CameraInfo cam_info;

std::string image_topic;
std::string point_cloud_topic;
std::string camera_info_topic;
bool use_depth;
bool received_cam_info = false;
bool received_point_cloud = false;

static const double OUT_OF_RANGE_DEPTH = 3.0;


float clamp(float value, float min, float max)
{
    return value < min ? min : (value > max ? max : value);
}

void update_visualization(Mat image, std_msgs::Header header, int x, int y)
{
    circle(image, Point(x, y), 5, CV_RGB(0,0,255)); //TODO: replace with fillPoly
    cv_bridge::CvImage viz_msg;
    viz_msg.header = header;
    viz_msg.encoding = sensor_msgs::image_encodings::BGR8;
    viz_msg.image = image;
    image_viz_pub.publish(viz_msg.toImageMsg());
}

geometry_msgs::Point get_average_point(PointCloud cloud, int u, int v, int radius, int threshold)
{
    geometry_msgs::Point average_point;
    pcl::PointXYZRGB cloud_point;
    double fx, fy, cx, cy, depth_sum;
    int width, height, u_start, u_end, v_start, v_end, num_valid_depths;

    // Find
    fx = cam_info.P[0];
    fy = cam_info.P[5];
    cx = cam_info.P[2];
    cy = cam_info.P[6];
    average_point.y = -((u - cx) / fx) * OUT_OF_RANGE_DEPTH;
    average_point.z = -((v - cy) / fy) * OUT_OF_RANGE_DEPTH;

    //Try to find depth value in range
    width = cloud.width;
    height = cloud.height;
    u_start = clamp(u - radius, 0, width);
    u_end = clamp(u + radius, 0, height);
    v_start = clamp(v - radius, 0, height);
    v_end = clamp(v + radius, 0, height);

    depth_sum = 0.0;
    num_valid_depths = 0;

    for(int i = u_start; i < u_end; i++)
    {
        for(int j = v_start; j < v_end; j++)
        {
            cloud_point = cloud.at(i, j);

            if(cloud_point.x > 0.0)
            {
                depth_sum += cloud_point.x;
                num_valid_depths++;
            }
        }
    }

    if(num_valid_depths < threshold)
    {
        average_point.x = depth_sum / num_valid_depths;
    }
    else
    {
        average_point.x = OUT_OF_RANGE_DEPTH;
    }

    return average_point;
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    point_cloud_msg = *msg;
    received_point_cloud = true;
}

void cam_info_callback(const sensor_msgs::CameraInfoPtr& msg)
{
    cam_info = *msg;
    sub_cam_info.shutdown();
    received_cam_info = true;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if(use_depth && !received_cam_info)
    {
        ROS_WARN("Waiting to receive CameraInfo from %s topic", camera_info_topic.c_str());
        return;
    }
    else if(use_depth && received_cam_info && !received_point_cloud)
    {
        ROS_WARN("Waiting to receive PointCloud from %s topic", point_cloud_topic.c_str());
        return;
    }

    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        image = cv_ptr->image;

        // Resize image
        double ratio = im_size.width * 1. / image.cols;
        resize(image, resized_image, Size(0,0), ratio, ratio, INTER_NEAREST);

        // Find salient point
        vector<KeyPoint> pts;
        salTracker.detect(resized_image, pts);
        salTracker.getSalImage(sal);

        double min, max;
        Point minloc, maxloc;
        minMaxLoc(sal, &min, &max, &minloc, &maxloc);

        lqrpt[0] = maxloc.x * 1.0 / sal.cols;
        lqrpt[1] = maxloc.y * 1.0 / sal.rows;

        salientSpot.setTrackerTarget(lqrpt);
        salientSpot.updateTrackerPosition();
        lqrpt = salientSpot.getCurrentPosition();

        // Image coords in resized image
        int im_x = lqrpt[0] * resized_image.cols;
        int im_y = lqrpt[1] * resized_image.rows;

        if(use_depth)
        {
            // Convert to cloud
            PointCloud cloud;
            pcl::PCLPointCloud2 cloud_2;
            pcl_conversions::toPCL(point_cloud_msg, cloud_2);
            pcl::fromPCLPointCloud2(cloud_2, cloud);

            // Get image coordinates
            double u, v;
            u = lqrpt[0] * cloud.width;
            v = lqrpt[1] * cloud.height;
            int threshold = 1;

            // See if there's a point in the region
            geometry_msgs::Point point = get_average_point(cloud, u, v, radius, threshold);

            geometry_msgs::PointStamped point_stamped;
            point_stamped.header.stamp = msg->header.stamp;
            point_stamped.header.frame_id = point_cloud_msg.header.frame_id;
            point_stamped.point = point;
            sal_3d_pub.publish(point_stamped);
        }
        else
        {
            SalientPoint2D point;
            point.header.stamp = msg->header.stamp;
            point.header.frame_id = msg->header.frame_id;
            point.image_width = resized_image.cols;
            point.image_height = resized_image.rows;
            point.x = im_x;
            point.y = im_y;
            sal_2d_pub.publish(point);
        }

        if(visualize)
        {
            update_visualization(resized_image, msg->header, im_x, im_y);
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

    // Load parameters
	nhr.param<string>("image", image_topic, "/softkinetic_camera/rgb_data");
	nhr.param<string>("point_cloud", point_cloud_topic, "");
	nhr.param<string>("camera_info", camera_info_topic, "");
	nhr.param<bool>("visualize", visualize, true);
	nhr.param<bool>("use_depth", use_depth, false);

	ROS_INFO("Image: %s", image_topic.c_str());
	ROS_INFO("Point cloud: %s", point_cloud_topic.c_str());
	ROS_INFO("Camera info: %s", camera_info_topic.c_str());
	ROS_INFO("Use depth: %s", use_depth ? "true" : "false");
	ROS_INFO("Visualisation: %s", visualize ? "true" : "false");
	ROS_INFO("Radius: %d", radius);

    // Set up publishers and subscribers
    sub_image = it.subscribe(image_topic, 1, image_callback);

	if(use_depth)
	{
        sub_pcl = nh.subscribe(point_cloud_topic, 1, point_cloud_callback);
        sub_cam_info = nh.subscribe(camera_info_topic, 1, cam_info_callback);
        sal_3d_pub = nh.advertise<geometry_msgs::PointStamped>("/salient_point_3d", 1);
	}
	else
	{
	    sal_2d_pub = nh.advertise<SalientPoint2D>("/salient_point_2d", 1);
	}

	if(visualize)
	{
        image_viz_pub = nh.advertise<sensor_msgs::Image>("/salient_point_viz", 1);
	}

	ros::spin();
}
