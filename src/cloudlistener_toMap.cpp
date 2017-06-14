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
ros::Subscriber sub,dub ;
tf::TransformListener *tf_listener; 
PointCloud depth_in;


void callback(const PointCloud::ConstPtr& pcl_in)
{
	//this node gets camera's pointcloud, transforms it at global world parameters and publishes it to the slammin node..
	PointCloud pcl_out,pca;

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
	
	pcl_ros::transformPointCloud("/world", *pcl_in, pcl_out, *tf_listener);	
	
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
	for (int i = 0; i < pcl_out.points.size(); ++i)
	{
		//locates pointcloud points above the ground level..
		if(!isnan(pcl_out.points[i].z)&&pcl_out.points[i].z>0.1){ //~~~>add launch parameter here for min heigth
			p_.x=pcl_out.points[i].x;
			p_.y=pcl_out.points[i].y;
			p_.z=pcl_out.points[i].z;
			p_.posIncloud=i;
			v_.vec3d.push_back(p_);
			//ROS_INFO("edw vrika ena %f %f %f",p_.x,p_.y,p_.z);
		}else if(isnan(pcl_out.points[i].z)) howmanynan++;
	}


	//ROS_INFO("tHA STEILEi nan %d kai to megethos einai %d",howmanynan,v_.vec3d.size());
	// for (int i = 0; i < v_.vec3d.size(); ++i)
	// {
	// 	if(isnan(v_.vec3d[i].x)||isnan(v_.vec3d[i].y)||isnan(v_.vec3d[i].z)){
	// 		printf("nan er\n");
	// 	}
	// }

	pV_pub.publish(v_);

	PointCloud depth_out;
	pcl_ros::transformPointCloud("/world", pcl_out, depth_in, *tf_listener);	
	pcl_ros::transformPointCloud("/base_link", depth_in, depth_out, *tf_listener);	
	ROS_INFO("TO PRWTO arxiko %f %f %f",depth_in.points[0].x,depth_in.points[0].y,depth_in.points[0].z);
	ROS_INFO("TEOLOS px des ayto %f %f %f",depth_out.points[0].x,depth_out.points[0].y,depth_out.points[0].z);
	depth_in.points.clear();

/*	for (int i = 0; i < depth_out.points.size(); ++i)
	{


		ROS_INFO("px des ayto %f %f %f",depth_out.points[i].x,depth_out.points[i].y,depth_out.points[i].z);
	}*/

/*	int count5=0,count6=0;
			if(p_.x>6)
				count6++;
			else if(p_.x>5)
				count5++;*/

	//ROS_INFO("ta 6 einai %d ta 5 einai %d",count6,count5);
}

void depthcam_scanCallback(const slammin::pointVector3d::ConstPtr& data){
	//depthPoints=*data;
/*	pcl::PointXYZRGB p_;
	depth_in.points.push_back(p_);*/
	ROS_INFO("elave %d",data->vec3d.size());
	for (int i = 0; i < depth_in.points.size(); ++i)
	{
		depth_in.points[i].x=0;
		depth_in.points[i].y=0;
		depth_in.points[i].z=0;

	}

	for (int i = 0; i < data->vec3d.size(); ++i)
  	{
		depth_in.points[i].x=data->vec3d[i].x;
		depth_in.points[i].y=data->vec3d[i].y;
		depth_in.points[i].z=data->vec3d[i].z;
  	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloudlistener_toMap");
  ros::NodeHandle nh;
  tf_listener    = new tf::TransformListener();
  sub= nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
	dub= nh.subscribe<slammin::pointVector3d>("/depthcam_scan", 1, depthcam_scanCallback);

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

