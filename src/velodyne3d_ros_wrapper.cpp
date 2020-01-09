/*
 * velodyne3d_ros_wrapper.cpp
 *
 *  Created on: Dec 16, 2019
 *      Author: jasmin
 */

#include "obcore/math/mathbase.h"
#include "obvision/reconstruct/space/SensorVelodyne3D.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"

obvious::SensorVelodyne3D *sensor;
obvious::TsdSpace *space;
ros::Publisher pubPulledOutCloud;

bool pulledOutCloud(void) {
  std::cout << "perform axisparallel raycast here and pub as pointcloud in rviz"
            << std::endl;

  unsigned int cellsX = space->getXDimension();
  unsigned int cellsY = space->getYDimension();
  unsigned int cellsZ = space->getZDimension();
  static obvious::obfloat*      coords  = new obvious::obfloat[cellsX * cellsY * cellsZ * 3];
  static obvious::obfloat*      normals = new obvious::obfloat[cellsX * cellsY * cellsZ * 3];
  static unsigned char*         rgb     = new unsigned char[cellsX * cellsY * cellsZ * 3];
  unsigned int                  cnt     = 0;
  static unsigned int           seq     = 0;
  obvious::RayCastAxisAligned3D raycaster;
  raycaster.calcCoords(space, coords, normals, rgb, &cnt);

  if(cnt == 0)
  return false;

  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  obvious::obfloat tr[3];
  space->getCentroid(tr);
  
  for(unsigned int i = 0; i < cnt; i += 3)
  {
    pcl::PointXYZRGBNormal p;
    p.x = coords[i]     - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];
    p.x = -p.x;

    p.normal_x = normals[i]     - tr[0];
    p.normal_y = normals[i + 1] - tr[1];
    p.normal_z = normals[i + 2] - tr[2];

    p.r = rgb[i];
    p.g = rgb[i + 1];
    p.b = rgb[i + 2];

    cloud.push_back(p);
  }
  cloud.header.frame_id = "map";
  cloud.header.seq      = seq++;
  pubPulledOutCloud.publish(cloud);
  return true;
}

void velodyneCallback(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  // IM CALLBACK ABFRAGEN OB SPACE SCHON INITIALISIERT IST --> INIT ROUTINE
  // MACHEN
  //_size aus SensorVelodyne3D = 1920
  std::vector<double> depthData(cloud.height * cloud.width, 0.0);
  unsigned int valid = 0;
  for (unsigned int i = 0; i < cloud.height; i++) {
    for (unsigned int j = 0; j < cloud.width; j++) {
      const unsigned int idx = i * cloud.width + j;
      double x = double(cloud.points[idx].x);

      // // double depth;
      // double depth = sqrt(cloud.points[idx].x * cloud.points[idx].x +
      //                     cloud.points[idx].y * cloud.points[idx].y +
      //                     cloud.points[idx].z * cloud.points[idx].z);
      //cast floats x y z to doubles
      double depth = sqrt(double(cloud.points[idx].x) * double(cloud.points[idx].x) +
                          double(cloud.points[idx].y) * double(cloud.points[idx].y) +
                          double(cloud.points[idx].z) * double(cloud.points[idx].z));
                          

      depthData[idx] = depth;
      valid++;
    }
  }
  std::cout << __PRETTY_FUNCTION__
            << "cloud.height*cloud.width = " << cloud.height * cloud.width
            << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "depthData.size() = " << depthData.size()
            << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points"
            << std::endl;

  sensor->setRealMeasurementData(depthData.data());
 //sensor->setRealMeasurementData();
  std::cout << "aaaaaaaa" << std::endl;
  space->push(sensor);
  std::cout << "bbbbbbbb" << std::endl;
  pulledOutCloud();

}

int main(int argc, char **argv) {

  ros::init(argc, argv, "velodyne3d_node");
  ros::NodeHandle nh;
  pubPulledOutCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("pulledOut_cloud", 1);

  // instantiate sensor
  unsigned int raysIncl = 16;
  double inclMin = obvious::deg2rad(-15.0);
  double inclRes = 2.0;
  double azimRes = 0.2;
  sensor = new obvious::SensorVelodyne3D(raysIncl, inclMin, inclRes, azimRes);
  // instantiate tsd space
  const double voxelSize = 0.01;
  space = new obvious::TsdSpace(voxelSize, obvious::LAYOUT_8x8x8,
                                obvious::LAYOUT_512x512x512);
  space->setMaxTruncation(3.0 * voxelSize);

  // transform sensor
  // 1 translation
  obvious::obfloat translation[3];
  space->getCentroid(translation);
  // translation[2] = 0.001;      //todo was macht May hier mit z? aus
  // tsd_test.cpp
  // 2 rotation about y-axis of sensor
  double theta = 0.0 * M_PI / 180;

  double transform[16] = {
      cos(theta),  0, sin(theta), translation[0], 0, 1, 0, translation[1],
      -sin(theta), 0, cos(theta), translation[2], 0, 0, 0, 1};
  obvious::Matrix MsensorTransform(4, 4);
  MsensorTransform.setData(transform);
  sensor->transform(&MsensorTransform);

  // NEXT STEP: SETREALMEASUREMENTDATA -- ASO BRAUCH ICH DIE ABSTANDSMESSUNGEN
  // IN EINEM DOUBLE ARRAY --> MUSS MIR DIE ABSTÄNDE AUS DEN 3d PUNKTEN
  // AUSRECHNEN
  // WEIL ICH DIREKT ÜBER DIE ROS POINTCLOUD GEHE UND NICHT ÜBER DIE VELODYNE
  // RAW DATA
  // DANN push methode aufrufen --> die ruft wiederum backproject aus dem
  // sensormodell auf!s

  ros::Subscriber subVelodyne =
      nh.subscribe("velodyne_points", 1000, velodyneCallback);
  ros::spin();
}
