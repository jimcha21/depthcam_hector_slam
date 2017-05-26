// some libs need to be removed**
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <stdlib.h>  
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/visualization/cloud_viewer.h>  
#include "slammin/pointVector3d.h"
#include "slammin/point3d.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Publisher pV_pub;
ros::Subscriber sub ;
tf::TransformListener *tf_listener; 

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
	//printf("fetched %d points\n",pcl_out.points.size() );
	
	slammin::pointVector3d v_;
	slammin::point3d p_;
	ROS_INFO("the size is %d",pcl_out.points.size());
	int howmanynan=0;
	for (int i = 0; i < pcl_out.points.size(); ++i)
	{
		if(!isnan(pcl_out.points[i].z)&&pcl_out.points[i].z>0.1){ //~~~>add launch parameter here for min heigth
			p_.x=pcl_out.points[i].x;
			p_.y=pcl_out.points[i].y;
			p_.z=pcl_out.points[i].z;
			p_.posIncloud=i;
			v_.vec3d.push_back(p_);
		}else if(isnan(pcl_out.points[i].z)) howmanynan++;

	}

	// for (int i = 0; i < v_.vec3d.size(); ++i)
	// {
	// 	if(isnan(v_.vec3d[i].x)||isnan(v_.vec3d[i].y)||isnan(v_.vec3d[i].z)){
	// 		printf("nan er\n");
	// 	}
	// }

	pV_pub.publish(v_);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloudlistener_toMap");
  ros::NodeHandle nh;
  tf_listener    = new tf::TransformListener();
  sub= nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  pV_pub = nh.advertise<slammin::pointVector3d> ("/slammin_pointVector3d", 1);

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

  ros::spin();
  return 0; 
}

