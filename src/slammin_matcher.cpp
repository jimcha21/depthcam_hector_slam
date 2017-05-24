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
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>

#define PI 3.14159265

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


//ros::Publisher pV_pub;
ros::Subscriber sub,map_sub,cam_sub,pose_sub;
ros::ServiceClient client;
nav_msgs::OccupancyGrid map_;
tf::TransformListener *tf_listener; 
geometry_msgs::Pose pose_;
slammin::pointVector3d mapV,mapC;
slammin::pointVector3d point_v_;
std::vector<int> indexes_vec;
float res=0.25;
int iterations=0;

void vector_data(const slammin::pointVector3d::ConstPtr& data)
{
	//ROS_INFO("received point_v_");
	point_v_=*data;
}

bool isIn(std::vector<int> ind_,int i_){
	for (int i = 0; i < ind_.size(); ++i)
	{
		if(ind_[i]==i_) return true;
	}
	return false;
}

void mapgrids_onRange_rec(float angle,float pos_x,float pos_y,float res){

	float newPos_x,newPox_y,x_fr_res,y_fr_res,x_lef_res,x_rig_res,y_lef_res,y_rig_res;
	
	slammin::point3d p_;
	//index->node id
	int index=(int)((pos_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((pos_x-map_.info.origin.position.x)/map_.info.resolution);
	
	
	//calculate 3d point distance from the robot, to check if this measurement is over the limit..
	float dist=sqrt(pow(pose_.position.x-pos_x,2)+pow(pose_.position.y-pos_y,2));

	//Recursion break~ 
	if(dist>9 || isIn(indexes_vec,index)|| angle!=angle){ //quit on invalid angle value (nan)
		// /if (isIn(indexes_vec,index)) ROS_INFO("revisited");
		return;
	}
	//iterations++; //debugin

	//mark as visited gridmap point
	indexes_vec.push_back(index);	

	//mporei na ginei kai ektos anadromis den alazei~stable O()
	if(angle>22.5&&angle<=67.5){ //45 deg
		x_fr_res=res;y_fr_res=res;  x_lef_res=0;y_lef_res=res;  x_rig_res=res;y_rig_res=0;
	}else if(angle>67.5&&angle<=112.5){ //90
		x_fr_res=0;y_fr_res=res;  x_lef_res=-res;y_lef_res=res;  x_rig_res=res;y_rig_res=res;
	}else if(angle>112.5&&angle<=157.5){ //135
		x_fr_res=-res;y_fr_res=res;  x_lef_res=-res;y_lef_res=0;  x_rig_res=0;y_rig_res=res;
	}else if(angle>157.5&&angle<=202.5){ //180
		x_fr_res=-res;y_fr_res=0;  x_lef_res=-res;y_lef_res=-res;  x_rig_res=-res;y_rig_res=res;
	}else if(angle>202.5&&angle<=247.5){ //225
		x_fr_res=-res;y_fr_res=-res;  x_lef_res=0;y_lef_res=-res;  x_rig_res=-res;y_rig_res=0;
	}else if(angle>247.5&&angle<=292.5){ //270
		x_fr_res=0;y_fr_res=-res;  x_lef_res=res;y_lef_res=-res;  x_rig_res=-res;y_rig_res=-res;
	}else if(angle>292.5&&angle<=337.5){ //315
		x_fr_res=res;y_fr_res=-res;  x_lef_res=res;y_lef_res=0;  x_rig_res=0;y_rig_res=-res;
	}else if(angle>337.5||angle<=22.5){ //360-0
		x_fr_res=res;y_fr_res=0;  x_lef_res=res;y_lef_res=res;  x_rig_res=res;y_rig_res=-res;
	}


	// newPos_x=cos(angle*PI/180)*pose_.position.x-sin(angle*PI/180)*pose_.position.y+x_fr_res;
	// newPox_y=sin(angle*PI/180)*pose_.position.x+cos(angle*PI/180)*pose_.position.y+y_fr_res;
	newPos_x=pos_x+x_fr_res;
	newPox_y=pos_y+y_fr_res;
	index=(int)((newPox_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((newPos_x-map_.info.origin.position.x)/map_.info.resolution);

	//ROS_INFO("the index is %d with coords %f %f me angle %f",index,newPos_x,newPox_y,angle);
	if(map_.data[index]==100){
		//ROS_INFO("found here! ");
		p_.x=newPos_x;
		p_.y=newPox_y;
		p_.z=0;
		p_.posIncloud=index;
		mapV.vec3d.push_back(p_);					
	}

	mapgrids_onRange_rec(angle,newPos_x,newPox_y,res);

	newPos_x=pos_x+x_lef_res;
	newPox_y=pos_y+y_lef_res;
	index=(int)((newPox_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((newPos_x-map_.info.origin.position.x)/map_.info.resolution);

	//ROS_INFO("the index is %d with coords %f %f me angle %f",index,newPos_x,newPox_y,angle);
	if(map_.data[index]==100){
		//ROS_INFO("found here! ");
		p_.x=newPos_x;
		p_.y=newPox_y;
		p_.z=0;
		p_.posIncloud=index;
		mapV.vec3d.push_back(p_);					
	}

	mapgrids_onRange_rec(angle,newPos_x,newPox_y,res);	

	newPos_x=pos_x+x_rig_res;
	newPox_y=pos_y+y_rig_res;
	index=(int)((newPox_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((newPos_x-map_.info.origin.position.x)/map_.info.resolution);

	//ROS_INFO("the index is %d with coords %f %f me angle %f",index,newPos_x,newPox_y,angle);
	if(map_.data[index]==100){
		//ROS_INFO("found here! ");
		p_.x=newPos_x;
		p_.y=newPox_y;
		p_.z=0;
		p_.posIncloud=index;
		mapV.vec3d.push_back(p_);					
	}

	mapgrids_onRange_rec(angle,newPos_x,newPox_y,res);		

	
//ROS_INFO("me cos %f sin %f einai sto %f %f kai tha paei sto %f %f ",cos(angle),sin(angle),pose_.position.x,pose_.position.y,mprostatoux,mprostatouy);

	// for (int i = 0; i <map_.info.height*map_.info.width ; ++i) //map_->info.height*map_->info.width
	// {
	// 	slammin::point3d p_;

	// 	//recostruction of the 2d map in world coordinates..
	// 	if(map_.data[i]==100){
	// 		// /ROS_INFO("%d",map_.data[i]);
	// 		p_.x=(div(i,map_.info.height).rem)*map_.info.resolution + map_.info.origin.position.x;
	// 		p_.y=(div(i,map_.info.height).quot)*map_.info.resolution + map_.info.origin.position.y;
	// 		float index=((p_.y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((p_.x-map_.info.origin.position.x)/map_.info.resolution);
	// 		float da=(float)(((float)(4-3)/(float)2)*(float)4);
	// 		ROS_INFO("see that %d %d %f",(int)index,i,da);
	// 		//p_.z=0;
	// 		//p_.posIncloud=0; //in mapV is useless, so it will be used in the recursive func
	// 		//mapV.vec3d.push_back(p_);
	// 	}
	// }
	return;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
  	cv_bridge::CvImagePtr cv_ptr;
  	
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
	}
	// cv::imshow("OPENCV_WINDOW", cv_ptr->image);
	// cv::waitKey(3);

	slammin::pointVector3d matched_v_;
	slammin::point3d matched_p_;
	int iters=0;
	// for (int i = 0; i < point_v_.vec3d.size(); ++i)
	// {
	// 	for (int j = 0; j < mapV.vec3d.size(); ++j)
	// 	{
	// 		iters++;
	// 		if ((std::abs(mapV.vec3d[j].x-point_v_.vec3d[i].x)<=0.1) && (std::abs(mapV.vec3d[j].y-point_v_.vec3d[i].y)<=0.1)){
	// 			//matched_p_=point_v_.vec3d[i];
	// 			matched_p_.x=div(point_v_.vec3d[i].posIncloud,640).rem; //matched_p_.x=point_v_.vec3d[i].x;
	// 			matched_p_.y=div(point_v_.vec3d[i].posIncloud,640).quot;//matched_p_.y=point_v_.vec3d[i].y;
	// 			matched_p_.z=point_v_.vec3d[i].z; //height category 
	// 			matched_p_.posIncloud=point_v_.vec3d[i].posIncloud;
	// 			matched_v_.vec3d.push_back(matched_p_);
				
	// 			j=mapV.vec3d.size();
	// 		}

	// 	}

	// }


// ...
	tf::Quaternion q(pose_.orientation.x,pose_.orientation.y,pose_.orientation.z,pose_.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	float angle;
	angle=(180*std::abs(yaw))/3.10;
	if(yaw<0){
		angle=angle-180;		
		angle=180+std::abs(angle);
	}
	//ROS_INFO("angle %f",angle);

	iterations=0;
	//ROS_INFO("its in");
	if(mapC.vec3d.size()>0){
	mapgrids_onRange_rec(angle,pose_.position.x,pose_.position.y,res);}
	//ROS_INFO("its out and free %d %d",mapV.vec3d.size(),iterations);
ROS_INFO("matched size  %d with filtered %d",mapC.vec3d.size(),mapV.vec3d.size());
	indexes_vec.clear();
	mapV.vec3d.clear();
	// int thres=10000;
	// if(mapV.vec3d.size()>0){
	// 	ROS_INFO("tha ksekinisei %d kai me point 3 %d",mapV.vec3d.size(),point_v_.vec3d.size());
	// }else{
	// 	ROS_INFO("nope");
	// }
	// for (int i = 0; i < mapV.vec3d.size(); ++i)
	// {
	// 	int forthis=0;
	// 	for (int j = 0; j < point_v_.vec3d.size(); ++j)
	// 	{
	// 		iters++;
			
	// 		if ((std::abs(mapV.vec3d[i].x-point_v_.vec3d[j].x)<=0.3) && (std::abs(mapV.vec3d[i].y-point_v_.vec3d[j].y)<=0.3)){
	// 			//matched_p_=point_v_.vec3d[i];
	// 			matched_p_.x=div(point_v_.vec3d[j].posIncloud,160).rem; //matched_p_.x=point_v_.vec3d[i].x;
	// 			matched_p_.y=div(point_v_.vec3d[j].posIncloud,160).quot;//matched_p_.y=point_v_.vec3d[i].y;
	// 			matched_p_.z=point_v_.vec3d[j].z; //height category 
	// 			matched_p_.posIncloud=point_v_.vec3d[j].posIncloud;
	// 			matched_v_.vec3d.push_back(matched_p_);
	// 			forthis++;				
	// 		}

	// 		if(forthis>thres){
	// 			j=point_v_.vec3d.size();
	// 		}

	// 	}

	// }

	// ROS_INFO("num of matched ->%d and iters %d",matched_v_.vec3d.size(),iters);

	// // try
	// //pcl::toROSMsg (pcl_out, image_); //in case we had a pointcloud..
	// //cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
      
	// cv::Mat image = cv_ptr->image; 
	// for (int i = 0; i < matched_v_.vec3d.size(); ++i)
	// {
	// //if(matched_v_.vec3d[i].z==1){
	//   cv::Mat roi = image(cv::Rect(matched_v_.vec3d[i].x,matched_v_.vec3d[i].y,1, 1));
	//   cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 125)); 
	//   double alpha = 0.3;
	//   cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
	//  // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
	// //}
	// // else if(obj_px[i].z==2){
	// //   cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(0,255,0));
	// // }else if(obj_px[i].z==3){
	// //   cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(0,0,255));
	// // }
	// }

	// cv::imshow( "Image window", image);
	// cv::waitKey(3);   
}


void get_map_(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
	mapC.vec3d.clear();
	for (int i = 0; i <data->info.height*data->info.width ; ++i) //map_->info.height*map_->info.width
	{
		slammin::point3d p_;

		//recostruction of the 2d map in world coordinates..
		if(data->data[i]==100){
			// /ROS_INFO("%d",map_.data[i]);
			p_.x=(div(i,data->info.height).rem)*data->info.resolution + data->info.origin.position.x;
			p_.y=(div(i,data->info.height).quot)*data->info.resolution + data->info.origin.position.y;
			p_.z=0;
			p_.posIncloud=0; //in mapV is useless, so it will be used in the recursive func
			mapC.vec3d.push_back(p_);
		}
	}
	map_=*data;
	
}


void get_pose_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ps)
{
	//ROS_INFO("Received pose..");
	pose_.position.x=ps->pose.pose.position.x;
	pose_.position.y=ps->pose.pose.position.y;
	pose_.orientation.z=ps->pose.pose.orientation.z; 
	pose_.orientation.w=ps->pose.pose.orientation.w; 
	 //2d hector
	pose_.position.z=0;
	pose_.orientation.x=0;
	pose_.orientation.y=0;

}

void extract_map(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	ROS_INFO("Received new map...");
	slammin::pointVector3d map_vec_in2d;
	int a;
	for (int i = 0; i <10 ; ++i) //map->info.height*map->info.width
	{

		if(map->data[i]!=-1){
			ROS_INFO("%d",map->data[i]);
		}
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "slammin_matcher");
	ros::NodeHandle nh;
	//while(nh.ok()){ 	
		sub= nh.subscribe<slammin::pointVector3d> ("/slammin_pointVector3d", 1, vector_data);
		pose_sub=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/poseupdate", 1, get_pose_);
		map_sub= nh.subscribe<nav_msgs::OccupancyGrid> ("/dynamic_map", 1, get_map_);
		//map_sub= nh.subscribe<nav_msgs::OccupancyGrid> ("/map", 1, extract_map);
		cam_sub= nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 1, imageCb);
		//pV_pub = nh.advertise<slammin::pointVector3d> ("/slammin_pointVector3d", 1);

		// client = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
		// nav_msgs::GetMap srv;
		// mapC.vec3d.clear(); //clear previously cached map data..
		// if (client.call(srv))
		// {
		// 	map_= srv.response.map;		
		// 	for (int i = 0; i <map_.info.height*map_.info.width ; ++i) //map_->info.height*map_->info.width
		// 	{
		// 		slammin::point3d p_;

		// 		//recostruction of the 2d map in world coordinates..
		// 		if(map_.data[i]==100){
		// 			// /ROS_INFO("%d",map_.data[i]);
		// 			p_.x=(div(i,map_.info.height).rem)*map_.info.resolution + map_.info.origin.position.x;
		// 			p_.y=(div(i,map_.info.height).quot)*map_.info.resolution + map_.info.origin.position.y;
		// 			p_.z=0;
		// 			p_.posIncloud=0; //in mapV is useless, so it will be used in the recursive func
		// 			mapC.vec3d.push_back(p_);
		// 		}
		// 	}

		// }
		// else
		// {
		// 	ROS_ERROR("Failed to fetch the map");
		// 	return 1;
		// }

		// ROS_INFO("matched size  %d with filtered %d",mapC.vec3d.size(),mapV.vec3d.size());
		ros::spin();
  	//}
 
  return 0; 
}
