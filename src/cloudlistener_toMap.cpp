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
	//this node gets camera's pointcloud, transforms it at global world parameters and publishes it to the slammin node..
	PointCloud pcl_out,pcl_out2,pca;

	tf::StampedTransform transform;
	try
	{
		tf_listener->lookupTransform ("/world", pcl_in->header.frame_id, fromPCL(pcl_in->header).stamp, transform);
	}
	catch (tf::LookupException &e)
	{
		return;
	}
	catch (tf::ExtrapolationException &e)
	{
		//ROS_ERROR ("%s", e.what ());
		return;
	}
	
	//ROS_INFO("fetched pointcloud");
	for (int i = 0; i < pcl_in->points.size(); ++i)
	{
		//locates pointcloud points above the ground level..
		if(!isnan(pcl_in->points[i].z)&&pcl_in->points[i].z>0.1){ //~~~>add launch parameter here for min heigth
			//ROS_INFO("edw vrika ena %f %f %f",pcl_in->points[i].x,pcl_in->points[i].y,pcl_in->points[i].z);
		}

	}

	pcl_ros::transformPointCloud("/world", *pcl_in, pcl_out, *tf_listener);	
	pcl_ros::transformPointCloud("/base_link", pcl_out, pcl_out2, *tf_listener);	
	// pcl_ros::transformPointCloud("/base_footprint", pcl_out, pca, *tf_listener);
	// tf_listener->lookupTransform ("/base_footprint", "/world", ros::Time(0), transform);
	// tf::Vector3 acet(transform * tf::Vector3(pcl_out.points[1440].x, pcl_out.points[1440].y, pcl_out.points[1440].z));
	// ROS_INFO("whazaaa2 %f %f %f %d",pca.points[1440].x,pca.points[1440].y,pca.points[1440].z,pca.points.size());
	// ROS_INFO("whazaaup %f %f %f %d",acet.getX(),acet.getY(),acet.getZ(),pca.points.size());
	
	//tf_pub.publish(pcl_out);
	//printf("fetched %d points\n",pcl_out.points.size() );
	
	slammin::pointVector3d v_;
	slammin::point3d p_;

	int howmanynan=0;
	ROS_INFO("======================================");
	int count5=0,count6=0;
	for (int i = 0; i < pcl_out2.points.size(); ++i)
	{
		//locates pointcloud points above the ground level..
		if(!isnan(pcl_out2.points[i].z)&&pcl_out2.points[i].z>0.1){ //~~~>add launch parameter here for min heigth
			p_.x=pcl_out2.points[i].x;
			p_.y=pcl_out2.points[i].y;
			p_.z=pcl_out2.points[i].z;
			p_.posIncloud=i;
			v_.vec3d.push_back(p_);
			//ROS_INFO("edw vrika ena %f %f %f",p_.x,p_.y,p_.z);
			if(p_.x>6)
				count6++;
			else if(p_.x>5)
				count5++;
		}else if(isnan(pcl_out2.points[i].z)) howmanynan++;
	}

	ROS_INFO("ta 6 einai %d ta 5 einai %d",count6,count5);

	//ROS_INFO("tHA STEILEi nan %d kai to megethos einai %d",howmanynan,v_.vec3d.size());
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

