/*
 * main.cpp
 *
 *  Created on: Mar 19, 2020
 *      Author: jasmin
 */

#include "VelodyneTsd.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyneTsd_node");
  VelodyneTsd mapper(10.0, 10.0, 10.0, 0.025);
  ros::spin();
}