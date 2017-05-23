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
#include "pcl_ros/Vec.h"
#include "pcl_ros/Point.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef geometry_msgs::Point32 VP;
//typedef sensor_msgs::PointCloud2 PointCloud;

ros::Publisher tf_pub;
ros::Publisher ima,gridis;
pcl_ros::Vec mapCoords;
tf::TransformListener *tf_listener; 

sensor_msgs::Image image_; //cache the image message

// void draw_cloud(
//     const std::string &text,
//     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
// {
//     pcl::visualization::CloudViewer viewer(text);
//     viewer.showCloud(cloud);
//     while (!viewer.wasStopped())
//     {
//     }
// }

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr img_to_cloud(
//         const cv::Mat& image,
//         const cv::Mat &coords)
// {
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//     for (int y=0;y<image.rows;y++)
//     {
//         for (int x=0;x<image.cols;x++)
//         {
//             pcl::PointXYZRGB point;
//             point.x = coords.at<double>(0,y*image.cols+x);
//             point.y = coords.at<double>(1,y*image.cols+x);
//             point.z = coords.at<double>(2,y*image.cols+x);

//             cv::Vec3b color = image.at<cv::Vec3b>(cv::Point(x,y));
//             uint8_t r = (color[2]);
//             uint8_t g = (color[1]);
//             uint8_t b = (color[0]);

//             int32_t rgb = (r << 16) | (g << 8) | b;
//             point.rgb = *reinterpret_cast<float*>(&rgb);

//             cloud->points.push_back(point);
//         }
//     }
//     return cloud;
// }

// void cloud_to_img(
//         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
//         cv::Mat &coords,
//         cv::Mat &image)
// {
//     coords = cv::Mat(3, cloud->points.size(), CV_64FC1);
//     image = cv::Mat(480, 640, CV_8UC3);
//     for(int y=0;y<image.rows;y++)
//     {
//         for(int x=0;x<image.cols;x++)
//         {
//             coords.at<double>(0,y*image.cols+x) = cloud->points.at(y*image.cols+x).x;
//             coords.at<double>(1,y*image.cols+x) = cloud->points.at(y*image.cols+x).y;
//             coords.at<double>(2,y*image.cols+x) = cloud->points.at(y*image.cols+x).z;

//             cv::Vec3b color = cv::Vec3b(
//                     cloud->points.at(y*image.cols+x).b,
//                     cloud->points.at(y*image.cols+x).g,
//                     cloud->points.at(y*image.cols+x).r);

//             image.at<cv::Vec3b>(cv::Point(x,y)) = color;
//         }
//     }
// }

void callback2(const pcl_ros::Vec::ConstPtr& data)
{
  ROS_INFO("geia");
  mapCoords=*data;
}


void callback(const PointCloud::ConstPtr& pcl_in)
{

  PointCloud pcl_out;

  tf_listener->waitForTransform("/world", "/camera_depth_frame",ros::Time(0), ros::Duration(10.0));
  pcl_ros::transformPointCloud("/world", *pcl_in, pcl_out, *tf_listener);
  tf_pub.publish(pcl_out);
  // pcl::PCLPointCloud2 a;
  // pcl::fromPCLPointCloud2(a,*pcl_out);

  //printf ("Cloud: width = %d, height = %d\n %d", pcl_out.width, pcl_out.height,pcl_out.points.size());
  std::vector<geometry_msgs::Point32> obj_px;
   pcl_ros::Vec tosend;
  pcl_ros::Vec above;
  int j=0,o=0;
  bool found=true;
  float max_z=0,max_x=0,max_y=0;
  int at_i=0;
  if (pcl_out.points.size()>0){
    for (int i = 0; i < pcl_out.points.size(); ++i)//
    {
      pcl_ros::Point a;
      a.x=pcl_out.points[i].x;
      a.y=pcl_out.points[i].y;

      if (pcl_out.points[i].z==pcl_out.points[i].z && pcl_out.points[i].z>0.1)
        {
          j++;
          if(pcl_out.points[i].z>max_z){
            max_z=pcl_out.points[i].z;
            max_x=pcl_out.points[i].x;
            max_y=pcl_out.points[i].y;
            at_i=i;
          }
          // a.z=pcl_out.points[i].z;
          a.z=i;
          above.Vec.push_back(a);

          // if (found){
          //   o=i-220*640;
          //   //printf("to ipsos touo %f\n",pcl_out.points[i].z);
          //   found=false;
          // }
        }else a.z=0;
        tosend.Vec.push_back(a);

    }
    gridis.publish(tosend);
    printf("brike %d\n",j );
    printf("max %f\n",max_z );
    int er=0;
    // for (int i = 0; i < pcl_out.points.size(); ++i)//
    // {
    //   //printf("gia to %d , %f\n",i,std::abs(max_z-pcl_out.points[i].z+er) );
    //   if (pcl_out.points[i].z==pcl_out.points[i].z){//
    //     if(std::abs(max_z-pcl_out.points[i].z-er)<0.1*max_z)
    //     {
    //       geometry_msgs::Point32 p;
    //       p.x=div(i,640).rem;p.y=div(i,640).quot;p.z=1;
    //       obj_px.push_back(p);
    //     }
    //   else if (std::abs(max_z-pcl_out.points[i].z+er)<0.2*max_z)
    //     {
    //       geometry_msgs::Point32 p;
    //       p.x=div(i,640).rem;p.y=div(i,640).quot;p.z=2;
    //       obj_px.push_back(p);
    //     }
    //   else if (std::abs(max_z-pcl_out.points[i].z+er)<0.3*max_z)
    //     {
    //       geometry_msgs::Point32 p;
    //       p.x=div(i,640).rem;p.y=div(i,640).quot;p.z=3;
    //       obj_px.push_back(p);
    //     }
    //   }
    // }

    printf("mapCoords size %d tosend size %d\n",mapCoords.Vec.size(),above.Vec.size() );
    printf("sto se seira px=%d ,to j einai %d me max %f %f %f kai i %d\n",o,j,max_x,max_y,max_z,at_i );
    printf("to size twn taksinomimenwn pixel einai %d\n",obj_px.size() );
    int done=0,posa=0; bool vrike=false;
    if(mapCoords.Vec.size()>0){
      for (int i = 0; i < above.Vec.size(); ++i)
      {
       
        for (int j = 0; j < mapCoords.Vec.size(); ++j)
        {
          if ((std::abs(above.Vec[i].x-mapCoords.Vec[j].x)<=0.1) && (std::abs(above.Vec[i].y-mapCoords.Vec[j].y)<=0.1)){
            geometry_msgs::Point32 p;
            p.x=div(above.Vec[i].z,640).rem;p.y=div(above.Vec[i].z,640).quot;p.z=1;
            obj_px.push_back(p);
            done++;
            j=mapCoords.Vec.size();
            vrike=true;
          } 
        }
        if (!vrike){
          //printf("den vrike gia to %d!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",i );
          posa++;
        }vrike=false;
      }
    }
    printf("den vrike %d!!!!!!!!!!!!!!!!!!!!!!!\n",posa );
    printf("done %d\n",done );

    // BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_out.points)
    //   printf ("\t( %f)\n", pt.z);
cv_bridge::CvImagePtr cv_ptr;
// cv::Mat image2 ;
// cv::Mat coords(3, 480*640, CV_64FC1);
// for (int col = 0; col < coords.cols; ++col)
// {
//     coords.at<double>(0, col) = col % 480;
//     coords.at<double>(1, col) = col / 480;
//     coords.at<double>(2, col) = 10;
// }

  





    try
    {
      pcl::toROSMsg (pcl_out, image_); //convert the cloud
      //pcl_ros=PointCloudToImage
      cv_ptr = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
      // Draw an example circle on the video stream
      cv::Mat image = cv_ptr->image; 
      for (int i = 0; i < obj_px.size(); ++i)
      {
        if(obj_px[i].z==1){
          cv::Mat roi = image(cv::Rect(obj_px[i].x,obj_px[i].y,1, 1));
          cv::Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 125)); 
          double alpha = 0.3;
          cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi); 
         // cv::circle(cv_ptr->image, cv::Point(obj_px[i].x, obj_px[i].y), 1, CV_RGB(255,0,0));
        }
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
      ima.publish(image_);

      // cloud_to_img(pcl_out, coords, image2);
      // cv::imshow("returned", image);

      // cv::waitKey();
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    obj_px.clear();
  }

}

// void pixelTo3DPoint(const PointCloud::ConstPtr pCloud, const int u, const int v,  std::vector<geometry_msgs::Point> &vp)
// {
//   // get width and height of 2D point cloud data
//   int width = pCloud->width;
//   int height = pCloud->height;

//   // Convert from u (column / width), v (row/height) to position in array
//   // where X,Y,Z data starts
//   int arrayPosition = v*pCloud->row_step + u*pCloud->point_step;

//   // compute position in array where x,y,z data start
//   int arrayPosX = arrayPosition + pCloud->fields[0].offset; // X has an offset of 0
//   int arrayPosY = arrayPosition + pCloud->fields[1].offset; // Y has an offset of 4
//   int arrayPosZ = arrayPosition + pCloud->fields[2].offset; // Z has an offset of 8

//  //  float X = 0.0;
//  //  float Y = 0.0;
//  //  float Z = 0.0;

//  //  memcpy(&X, &pCloud->data[arrayPosX], sizeof(float));
//  //  memcpy(&Y, &pCloud->data[arrayPosY], sizeof(float));
//  //  memcpy(&Z, &pCloud->data[arrayPosZ], sizeof(float));

//  // // put data into the point p
//  //  geometry_msgs::Point p;
//  //  p.x = X;
//  //  p.y = Y;
//  //  p.z = Z;
//  //  if ((p.x==p.x) && (p.y==p.y) && (p.z==p.z)) {
//  //    printf("to pix at %d %d\n",u,v );
//  //    printf("ston xwro %f %f %f\n",p.x,p.y,p.z );
//  //    //vp.push_back(p);
//  //  }
  
//   //printf("%f %f %f\n",X,Y,Z);

// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  ros::Subscriber sub2 = nh.subscribe<pcl_ros::Vec>("/mapV", 1, callback2);
 
  tf_pub = nh.advertise<PointCloud> ("tf_points2", 1);
  ima = nh.advertise<sensor_msgs::Image> ("ima", 1);
  gridis = nh.advertise<pcl_ros::Vec>("for_grid_map", 1);
  tf_listener    = new tf::TransformListener();

  ros::spin();
  //delete tf_listener; 
  //return 0; 
}

