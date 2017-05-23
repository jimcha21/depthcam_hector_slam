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

//ros::Publisher pV_pub;
ros::Subscriber sub,map_sub,cam_sub;
tf::TransformListener *tf_listener; 
slammin::pointVector3d mapV;
slammin::pointVector3d point_v_;
cv_bridge::CvImagePtr cv_ptr;

void vector_data(const slammin::pointVector3d::ConstPtr& data)
{
	//ROS_INFO("received point_v_");
	point_v_=*data;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
	for (int i = 0; i < point_v_.vec3d.size(); ++i)
	{
		for (int j = 0; j < mapV.vec3d.size(); ++j)
		{
			iters++;
			if ((std::abs(mapV.vec3d[j].x-point_v_.vec3d[i].x)<=0.1) && (std::abs(mapV.vec3d[j].y-point_v_.vec3d[i].y)<=0.1)){
				//matched_p_=point_v_.vec3d[i];
				matched_p_.x=div(point_v_.vec3d[i].posIncloud,640).rem; //matched_p_.x=point_v_.vec3d[i].x;
				matched_p_.y=div(point_v_.vec3d[i].posIncloud,640).quot;//matched_p_.y=point_v_.vec3d[i].y;
				matched_p_.z=point_v_.vec3d[i].z; //height category 
				matched_p_.posIncloud=point_v_.vec3d[i].posIncloud;
				matched_v_.vec3d.push_back(matched_p_);
				
				j=mapV.vec3d.size();
			}

		}

	}

	ROS_INFO("num of matched ->%d and iters %d",matched_v_.vec3d.size(),iters);

	// cv_bridge::CvImagePtr cv_ptr;
    // try
    // {
      //pcl::toROSMsg (pcl_out, image_); //in case we had a pointcloud..
      //pcl_ros=PointCloudToImage
      //cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
      
      cv::Mat image = cv_ptr->image; 
      for (int i = 0; i < matched_v_.vec3d.size(); ++i)
      {
        //if(matched_v_.vec3d[i].z==1){
          cv::Mat roi = image(cv::Rect(matched_v_.vec3d[i].x,matched_v_.vec3d[i].y,1, 1));
          cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 125)); 
          double alpha = 0.3;
          cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
         // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
        //}
        // else if(obj_px[i].z==2){
        //   cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(0,255,0));
        // }else if(obj_px[i].z==3){
        //   cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(0,0,255));
        // }
      }

      cv::imshow( "Image window", image);
      cv::waitKey(3);
      // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
 	//}     
}


void callback(const slammin::pointVector3d::ConstPtr& data)
{
	ROS_INFO("Received map with %d points...",data->vec3d.size());
	mapV=*data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slammin_matcher");
  ros::NodeHandle nh;

  sub= nh.subscribe<slammin::pointVector3d> ("/slammin_pointVector3d", 1, vector_data);
  map_sub= nh.subscribe<slammin::pointVector3d> ("/mapV", 1, callback);
  cam_sub= nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 1, imageCb);
  //pV_pub = nh.advertise<slammin::pointVector3d> ("/slammin_pointVector3d", 1);

  ros::spin();
  return 0; 
}
