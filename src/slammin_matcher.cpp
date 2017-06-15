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
#include <math.h>
#include <opencv2/core/core.hpp>
#include "slammin/pointVector3d.h"
#include "slammin/point3d.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>

#include <image_transport/image_transport.h>

#define PI 3.14159265

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


ros::Publisher depthmap_pub;
image_transport::Publisher img_pub;
ros::Subscriber sub,map_sub,cam_sub,pose_sub;
ros::ServiceClient client;
nav_msgs::OccupancyGrid map_;
tf::TransformListener *tf_listener; 
geometry_msgs::Pose pose_;
slammin::pointVector3d mapREC,mapC;
slammin::pointVector3d depthCamera_points;
std::vector<int> indexes_vec;

int iterations=0;
float max_z_point=0; // for height category clustering ..

bool visited(std::vector<int> ind_,int i_){
	for (int i = 0; i < ind_.size(); ++i)
	{
		if(ind_[i]==i_) return true;
	}
	return false;
}

//the recursive..
void mapgrids_onRange_rec(float angle,float pos_x,float pos_y){

	float newPos_x,newPos_y,x_fr_res,y_fr_res,x_lef_res,x_rig_res,y_lef_res,y_rig_res;
	float res=map_.info.resolution;
	slammin::point3d p_;
	//index->node id
	int index=(int)((pos_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((pos_x-map_.info.origin.position.x)/map_.info.resolution);
	
	
	//calculate 3d point distance from the robot, to check if this measurement is over the limit..
	float dist=sqrt(pow(pose_.position.x-pos_x,2)+pow(pose_.position.y-pos_y,2));

	//Recursion break~ 
	if(dist>9 || visited(indexes_vec,index)|| angle!=angle){ //quit on invalid angle value (nan)
		// /if (visited(indexes_vec,index)) ROS_INFO("revisited");
		return;
	}
	iterations++; //debugin

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
	// newPos_y=sin(angle*PI/180)*pose_.position.x+cos(angle*PI/180)*pose_.position.y+y_fr_res;
	newPos_x=pos_x+x_fr_res;
	newPos_y=pos_y+y_fr_res;
	index=(int)((newPos_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((newPos_x-map_.info.origin.position.x)/map_.info.resolution);

	//ROS_INFO("the index is %d with coords %f %f me angle %f",index,newPos_x,newPos_y,angle);
	if(map_.data[index]==100){
		//ROS_INFO("found here! ");
		p_.x=newPos_x;
		p_.y=newPos_y;
		p_.z=0;
		p_.posIncloud=index;
		mapREC.vec3d.push_back(p_);					
	}
	// for (int i = 0; i < mapC.vec3d.size(); ++i)
	// {
	// 	if (mapC.vec3d[i].posIncloud==index)
	// 	{
	// 		p_.x=newPos_x;
	// 		p_.y=newPos_y;
	// 		p_.z=0;
	// 		p_.posIncloud=index;
	// 		mapREC.vec3d.push_back(p_);	
	// 		i==mapC.vec3d.size();
	// 	}
	// }

	mapgrids_onRange_rec(angle,newPos_x,newPos_y);

	newPos_x=pos_x+x_lef_res;
	newPos_y=pos_y+y_lef_res;
	index=(int)((newPos_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((newPos_x-map_.info.origin.position.x)/map_.info.resolution);

	//ROS_INFO("the index is %d with coords %f %f me angle %f",index,newPos_x,newPos_y,angle);
	if(map_.data[index]==100){
		//ROS_INFO("found here! ");
		p_.x=newPos_x;
		p_.y=newPos_y;
		p_.z=0;
		p_.posIncloud=index;
		mapREC.vec3d.push_back(p_);					
	}
	// for (int i = 0; i < mapC.vec3d.size(); ++i)
	// {
	// 	if (mapC.vec3d[i].posIncloud==index)
	// 	{
	// 		p_.x=newPos_x;
	// 		p_.y=newPos_y;
	// 		p_.z=0;
	// 		p_.posIncloud=index;
	// 		mapREC.vec3d.push_back(p_);	
	// 		i==mapC.vec3d.size();
	// 	}
	// }
	mapgrids_onRange_rec(angle,newPos_x,newPos_y);	

	newPos_x=pos_x+x_rig_res;
	newPos_y=pos_y+y_rig_res;
	index=(int)((newPos_y-map_.info.origin.position.y)/map_.info.resolution)*map_.info.height+((newPos_x-map_.info.origin.position.x)/map_.info.resolution);

	//ROS_INFO("the index is %d with coords %f %f me angle %f",index,newPos_x,newPos_y,angle);
	if(map_.data[index]==100){
		//ROS_INFO("found here! ");
		p_.x=newPos_x;
		p_.y=newPos_y;
		p_.z=0;
		p_.posIncloud=index;
		mapREC.vec3d.push_back(p_);					
	}
	// for (int i = 0; i < mapC.vec3d.size(); ++i)
	// {
	// 	if (mapC.vec3d[i].posIncloud==index)
	// 	{
	// 		p_.x=newPos_x;
	// 		p_.y=newPos_y;
	// 		p_.z=0;
	// 		p_.posIncloud=index;
	// 		mapREC.vec3d.push_back(p_);	
	// 		i==mapC.vec3d.size();
	// 	}
	// }
	mapgrids_onRange_rec(angle,newPos_x,newPos_y);		

	
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
	// 		//p_.posIncloud=0; //in mapREC is useless, so it will be used in the recursive func
	// 		//mapREC.vec3d.push_back(p_);
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

	slammin::pointVector3d matched_points_,new_v_,som;
	slammin::point3d p_;
	int iters=0;
	// for (int i = 0; i < depthCamera_points.vec3d.size(); ++i)
	// {
	// 	for (int j = 0; j < mapREC.vec3d.size(); ++j)
	// 	{
	// 		iters++;
	// 		if ((std::abs(mapREC.vec3d[j].x-depthCamera_points.vec3d[i].x)<=0.1) && (std::abs(mapREC.vec3d[j].y-depthCamera_points.vec3d[i].y)<=0.1)){
	// 			//p_=depthCamera_points.vec3d[i];
	// 			p_.x=div(depthCamera_points.vec3d[i].posIncloud,640).rem; //p_.x=depthCamera_points.vec3d[i].x;
	// 			p_.y=div(depthCamera_points.vec3d[i].posIncloud,640).quot;//p_.y=depthCamera_points.vec3d[i].y;
	// 			p_.z=depthCamera_points.vec3d[i].z; //height category 
	// 			p_.posIncloud=depthCamera_points.vec3d[i].posIncloud;
	// 			matched_points_.vec3d.push_back(p_);
				
	// 			j=mapREC.vec3d.size();
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
	if(map_.info.width>0){//.vec3d.size()>0){
		mapgrids_onRange_rec(angle,pose_.position.x,pose_.position.y);
	}
	//ROS_INFO("its out and free %d %d",mapREC.vec3d.size(),iterations);
	//ROS_INFO("EKANE REC %d",iterations);
	ROS_INFO("Exhastive search matches and Recursive %d",/*mapC.vec3d.size(),*/mapREC.vec3d.size());
	//ROS_INFO("ayta p rthan %d",depthCamera_points.vec3d.size());
	
	int thres=1000000;
	
	for (int i = 0; i < depthCamera_points.vec3d.size(); ++i)
	{
		bool found=false;
		for (int j = 0; j < mapREC.vec3d.size(); ++j){
			iters++;
			if ((std::abs(mapREC.vec3d[j].x-depthCamera_points.vec3d[i].x)<=2*map_.info.resolution) && (std::abs(mapREC.vec3d[j].y-depthCamera_points.vec3d[i].y)<=2*map_.info.resolution)){
				found=true;
				p_.x=div(depthCamera_points.vec3d[i].posIncloud,cv_ptr->image.cols).rem; //p_.x=depthCamera_points.vec3d[i].x;
				p_.y=div(depthCamera_points.vec3d[i].posIncloud,cv_ptr->image.cols).quot;//p_.y=depthCamera_points.vec3d[i].y;
				p_.z=depthCamera_points.vec3d[i].z; //height category 
				p_.posIncloud=depthCamera_points.vec3d[i].posIncloud;

				//store highest scanned point
				if(p_.z>max_z_point){
					max_z_point=p_.z;
				}

				matched_points_.vec3d.push_back(p_);
				break;
			}
		}
		if(!found){
			p_.x=div(depthCamera_points.vec3d[i].posIncloud,cv_ptr->image.cols).rem; //p_.x=depthCamera_points.vec3d[i].x;
			p_.y=div(depthCamera_points.vec3d[i].posIncloud,cv_ptr->image.cols).quot;//p_.y=depthCamera_points.vec3d[i].y;
			p_.z=depthCamera_points.vec3d[i].z; //height category 
			p_.posIncloud=depthCamera_points.vec3d[i].posIncloud;
			new_v_.vec3d.push_back(p_);
			p_.x=depthCamera_points.vec3d[i].x; //p_.x=depthCamera_points.vec3d[i].x;
			p_.y=depthCamera_points.vec3d[i].y;//p_.y=depthCamera_points.vec3d[i].y;
			som.vec3d.push_back(p_);
		}
	}

	
	// for (int i = 0; i < mapREC.vec3d.size(); ++i)
	// {
	// 	int loopbreak=0;bool found=false;
	// 	for (int j = 0; j < depthCamera_points.vec3d.size(); ++j)
	// 	{
	// 		iters++;
			
	// 		if ((std::abs(mapREC.vec3d[i].x-depthCamera_points.vec3d[j].x)<=2*map_.info.resolution-0.1) && (std::abs(mapREC.vec3d[i].y-depthCamera_points.vec3d[j].y)<=2*map_.info.resolution-0.1)){
	// 			//p_=depthCamera_points.vec3d[i];
	// 			found=true;
	// 			p_.x=div(depthCamera_points.vec3d[j].posIncloud,cv_ptr->image.cols).rem; //p_.x=depthCamera_points.vec3d[i].x;
	// 			p_.y=div(depthCamera_points.vec3d[j].posIncloud,cv_ptr->image.cols).quot;//p_.y=depthCamera_points.vec3d[i].y;
	// 			p_.z=depthCamera_points.vec3d[j].z; //height category 
	// 			p_.posIncloud=depthCamera_points.vec3d[j].posIncloud;

	// 			if(p_.z>max_z_point){
	// 				max_z_point=p_.z;
	// 			}

	// 			matched_points_.vec3d.push_back(p_);
	// 			loopbreak++;				
	// 		}else{
	// 			p_.x=div(depthCamera_points.vec3d[j].posIncloud,cv_ptr->image.cols).rem; //p_.x=depthCamera_points.vec3d[i].x;
	// 			p_.y=div(depthCamera_points.vec3d[j].posIncloud,cv_ptr->image.cols).quot;//p_.y=depthCamera_points.vec3d[i].y;
	// 			p_.z=depthCamera_points.vec3d[j].z; //height category 
	// 			p_.posIncloud=depthCamera_points.vec3d[j].posIncloud;	
	// 			new_v_.vec3d.push_back(p_);
	// 		}

	// 		if(loopbreak>thres){
	// 			j=depthCamera_points.vec3d.size();
	// 		}

	// 	}


	// }

	ROS_INFO("Total %d matched, doing iters %d and found extra %d",matched_points_.vec3d.size(),iters,new_v_.vec3d.size());

	// // try
	// //pcl::toROSMsg (pcl_out, image_); //in case we had a pointcloud..
	// //cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
      
	cv::Mat image = cv_ptr->image; 
	double alpha = 0.5;
	for (int i = 0; i < matched_points_.vec3d.size(); ++i)
	{
		if(matched_points_.vec3d[i].z>=0.8*max_z_point){
		  cv::Mat roi =  image(cv::Rect(matched_points_.vec3d[i].x,matched_points_.vec3d[i].y,1, 1));
		  cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 0, 125)); 
		  cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
		 // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
		}else if(matched_points_.vec3d[i].z>=0.6*max_z_point){
		  cv::Mat roi =  image(cv::Rect(matched_points_.vec3d[i].x,matched_points_.vec3d[i].y,1, 1));
		  cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 62.5, 125)); 
		  cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
		 // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
		}else if(matched_points_.vec3d[i].z>=0.4*max_z_point){
		  cv::Mat roi =  image(cv::Rect(matched_points_.vec3d[i].x,matched_points_.vec3d[i].y,1, 1));
		  cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 125)); 
		  cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
		 // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
		}else{
		  cv::Mat roi =  image(cv::Rect(matched_points_.vec3d[i].x,matched_points_.vec3d[i].y,1, 1));
		  cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 0)); 
		  cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
		  // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
		}
	}
	//colorize the new discovered 3d points-obstacles
	for (int i = 0; i < new_v_.vec3d.size(); ++i)
	{
		if(i>20)
			break;
		if(new_v_.vec3d[i].x>=0&&new_v_.vec3d[i].y>=0){
			cv::Mat roi =  image(cv::Rect(new_v_.vec3d[i].x,new_v_.vec3d[i].y,1, 1));
			cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(125, 0, 0)); 
			cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
		}
	}

	indexes_vec.clear();
	mapREC.vec3d.clear();

	cv_bridge::CvImage out_msg;
	out_msg.header   = msg->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
	out_msg.image    =  cv_ptr->image; // Your cv::Mat

	
	img_pub.publish(out_msg.toImageMsg());
	//cv::imshow( "Image window",  image);
	//cv::waitKey(3);   

/*	for (int i = 0; i < som.vec3d.size(); ++i)
	{
		//ROS_INFO("another one %f %f",new_v_.vec3d[i].x,new_v_.vec3d[i].y);
		som.vec3d[i].x=ceil((som.vec3d[i].x+std::abs(map_.info.origin.position.x))*(1/map_.info.resolution));
		som.vec3d[i].y=ceil((som.vec3d[i].y+std::abs(map_.info.origin.position.y))*(1/map_.info.resolution));
	}*/
	depthmap_pub.publish(som);

}

void vector_data(const slammin::pointVector3d::ConstPtr& data)
{
	//ROS_INFO("received points_v_ %d", data->vec3d.size());
	depthCamera_points=*data;
}


void get_map_(const nav_msgs::OccupancyGrid::ConstPtr& data)
{
/*	mapC.vec3d.clear();
	int coun=0;
	for (int i = 0; i <data->info.height*data->info.width ; ++i) 
	{
		slammin::point3d p_;
		coun++;
		//recostruction of the 2d map in world coordinates..
		if(data->data[i]==100){
			// /ROS_INFO("%d",map_.data[i]);
			p_.x=(div(i,data->info.height).rem)*data->info.resolution + data->info.origin.position.x;
			p_.y=(div(i,data->info.height).quot)*data->info.resolution + data->info.origin.position.y;
			p_.z=0;
			p_.posIncloud=i; //in mapREC is useless, so it will be used in the recursive func
			mapC.vec3d.push_back(p_);
		}
	}*/
	map_=*data;
	//ROS_INFO("EKANE EXH %d",coun);
	//ROS_INFO("new map %d",mapC.vec3d.size());
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "slammin_matcher");
	ros::NodeHandle nh;

	//subscribers
	sub= nh.subscribe<slammin::pointVector3d> ("/slammin_pointVector3d", 1, vector_data);
	pose_sub=nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/poseupdate", 1, get_pose_);
	map_sub= nh.subscribe<nav_msgs::OccupancyGrid> ("/dynamic_map", 1, get_map_);
	cam_sub= nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 1, imageCb);
	
	//publishers
	image_transport::ImageTransport it(nh);
	img_pub = it.advertise("/camera/rgb/image_ra2w", 1);
	depthmap_pub=nh.advertise<slammin::pointVector3d> ("/depthcam_scan", 1);
	//pV_pub = nh.advertise<slammin::pointVector3d> ("/slammin_pointVector3d", 1);

	ros::spin();
 
  return 0; 
}
