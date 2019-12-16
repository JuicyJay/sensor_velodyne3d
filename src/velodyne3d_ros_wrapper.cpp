/*
 * velodyne3d_ros_wrapper.cpp
 *
 *  Created on: Dec 16, 2019
 *      Author: jasmin
 */

#include "ros/ros.h"
#include "obvision/reconstruct/space/SensorVelodyne3D.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "velodyne3d_node");
  unsigned int raysIncl = 16;
  double inclRes        = 2.0;
  double azimRes        = 3.0;
  //SensorVelodyne3D(unsigned int raysIncl, double inclMin, double inclRes, double azimRes, double maxRange=INFINITY, double minRange=0.0, double lowReflectivityRange=INFINITY);
  obvious::SensorVelodyne3D(raysIncl, inclRes, azimRes);
  ros::spin();
}


