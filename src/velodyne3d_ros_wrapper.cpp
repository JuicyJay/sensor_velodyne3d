/*
 * velodyne3d_ros_wrapper.cpp
 *
 *  Created on: Dec 16, 2019
 *      Author: jasmin
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "obvision/reconstruct/space/SensorVelodyne3D.h"
#include "obcore/math/mathbase.h"

void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::cout << __PRETTY_FUNCTION__ << "callback comin" << std::endl;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "velodyne3d_node");
  ros::NodeHandle n;

  ros::Subscriber subVelodyne = n.subscribe("velodyne_points", 1000, velodyneCallback);

  unsigned int raysIncl = 16;
  double inclMin        = obvious::deg2rad(-15.0);
  double inclRes        = 2.0;
  double azimRes        = 3.0;
  //SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange=INFINITY, double minRange=0.0, double lowReflectivityRange=INFINITY);
  obvious::SensorVelodyne3D(raysIncl, inclMin, inclRes, azimRes);
  ros::spin();
}


