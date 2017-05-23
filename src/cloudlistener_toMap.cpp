#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>
//#include "sensor_msgs/PointCloud2.h"
//#include "sensor_msgs/PointField.h"
#include "geometry_msgs/Point.h"
#include <vector>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <stdlib.h>  
#include <sensor_msgs/Image.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath>      
#include <opencv2/core/core.hpp>

#include "slammin/pointVector3d.h"
#include "slammin/point3d.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef geometry_msgs::Point32 VP;
//typedef sensor_msgs::PointCloud2 PointCloud;

ros::Publisher tf_pub;
ros::Publisher ima,gridis;
ros::Subscriber sub ;
tf::TransformListener *tf_listener; 

sensor_msgs::Image image_; //cache the image message


void callback(const PointCloud::ConstPtr& pcl_in)
{
	PointCloud pcl_out;
	tf::StampedTransform transform;
	try
	{
		tf_listener->lookupTransform ("/world", pcl_in->header.frame_id, fromPCL(pcl_in->header).stamp, transform);
	}
	catch (tf::LookupException &e)
	{
		return ;
	}
	catch (tf::ExtrapolationException &e)
	{
		//ROS_ERROR ("%s", e.what ());
		return;
	}
	
	//ROS_INFO("fetched pointcloud");
	pcl_ros::transformPointCloud("/world", *pcl_in, pcl_out, *tf_listener);
	//tf_pub.publish(pcl_out);
	printf("irthan %d\n",pcl_out.points.size() );
	slammin::pointVector3d v_;
	for (int i = 0; i < pcl_out.points.size(); ++i)
	{
		if(!isnan(pcl_out.points[i].z)){
			printf("helo\n");
		}
	}
	 
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloudlistener_toMap");
  ros::NodeHandle nh;
  tf_listener    = new tf::TransformListener();
  sub= nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);

  //ros::Subscriber sub2 = nh.subscribe<pcl_ros::Vec>("/mapV", 1, callback2);
 
  tf_pub = nh.advertise<PointCloud> ("tf_points2", 1);
  // ima = nh.advertise<sensor_msgs::Image> ("ima", 1);
  // gridis = nh.advertise<pcl_ros::Vec>("for_grid_map", 1);

	// ros::Rate rate(10.0);
	// while (nh.ok()){
	// 	tf::StampedTransform transform;
	// 	try {
	// 		tf_listener->waitForTransform("/world", "/camera_depth_frame", ros::Time(0), ros::Duration(1.0) );
	// 		//tf_listener->lookupTransform("/world", "/camera_depth_frame", ros::Time(0), transform);
	// 	} catch (tf::TransformException ex) {
	// 		ROS_ERROR("%s",ex.what());
	// 		continue;
	// 	}
	// 	sub= nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
	// 	rate.sleep();
	// }
    //return 0; 
  ros::spin();
}

