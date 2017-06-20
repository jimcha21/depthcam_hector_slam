#include "ros/ros.h"
#include "slammin/pointVector3d.h"
#include "slammin/point3d.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

nav_msgs::OccupancyGrid map_;
ros::Publisher map_pub;

int main(int argc, char** argv)
{

//this current node calls hector's slam dynamic_map service and publishes it's for the slammin_matcher node..
	ros::init(argc, argv, "map_advertiser");
	ros::NodeHandle nh;
	
	ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
	map_pub = nh.advertise<nav_msgs::OccupancyGrid> ("/dynamic_map", 1);
	
	nav_msgs::GetMap srv;
	
	while(nh.ok()){ 	

		if (client.call(srv))
		{
			map_= srv.response.map;		
			map_pub.publish(map_);
		}
		else
		{
			//ROS_ERROR("Failed to fetch the map, please run HectorMappingRos node...");
		}
  	}

	return 0;
}



