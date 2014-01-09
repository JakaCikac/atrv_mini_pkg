#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <iostream>
#include <string>

class ATRVFootprint {

public:
  ATRVFootprint();

private:
  void publishFootprint();
  geometry_msgs::PolygonStamped footprint_poly;
  ros::Publisher foot_pub;
  ros::NodeHandle ph, nh;
  boost::mutex publish_mutex_;

  ros::Timer timer_;
};

ATRVFootprint::ATRVFootprint():
  ph("~")
{
	foot_pub = ph.advertise<geometry_msgs::PolygonStamped>("footprint", 10);
	timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&ATRVFootprint::publishFootprint, this));
}
void ATRVFootprint::publishFootprint() {	
  //create a polygon message for the footprint
  
  footprint_poly.header.frame_id = "/base";
  footprint_poly.header.stamp = ros::Time::now();

  footprint_poly.polygon.points.resize(4);

  footprint_poly.polygon.points[0].x = -0.320;
  footprint_poly.polygon.points[0].y = 0.275;
  footprint_poly.polygon.points[0].z = 0;

  footprint_poly.polygon.points[1].x = 0.32;
  footprint_poly.polygon.points[1].y = 0.275;
  footprint_poly.polygon.points[1].z = 0;

  footprint_poly.polygon.points[2].x = 0.32;
  footprint_poly.polygon.points[2].y = -0.275;
  footprint_poly.polygon.points[2].z = 0;

  footprint_poly.polygon.points[3].x = -0.320;
  footprint_poly.polygon.points[3].y = -0.275;
  footprint_poly.polygon.points[3].z = 0;


  foot_pub.publish(footprint_poly);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ATRVmini_footprint");
  ATRVFootprint ATRV_footprint;
  ROS_INFO("Publishing footprint.");
  ros::spin();
}