/*
 * VelodyneTsd.h
 *
 *  Created on: Jan 7, 2020
 *      Author: jasmin
 */

#ifndef SRC_VELODYNETSD_H_
#define SRC_VELODYNETSD_H_

#include "obvision/reconstruct/space/RayCast3D.h"
// #include "obvision/reconstruct/space/SensorPolar3DBase.h"
#include "obvision/reconstruct/space/SensorVelodyne3D.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include "sensor_velodyne3d/VelodyneTsdReconfigureConfig.h"
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class VelodyneTsd
{
public:
  VelodyneTsd(const float dimX, const float dimY, const float dimZ, const float cellSize);
  virtual ~VelodyneTsd();
  void callbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  bool pubSensorRaycast();
  // void matchScanToSpace(const pcl::PointCloud<pcl::PointXYZ>& currentLaserSceneData, const pcl::PointCloud<pcl::PointXYZ>& lastRaycastModelData);
  void renderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

private:
  void                               init(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void                               callbackDynReconf(sensor_velodyne3d::VelodyneTsdReconfigureConfig& config, uint32_t level);
  std::unique_ptr<obvious::TsdSpace> _space;

  // Switch sensor class here
  std::unique_ptr<obvious::SensorVelodyne3D> _sensor;
  // std::unique_ptr<obvious::SensorPolar3DBase> _sensor;

  obvious::RayCast3D*                    _raycaster;
  ros::NodeHandle                        _nh;
  ros::Subscriber                        _subPointCloud;
  ros::Publisher                         _pubSensorRaycast;
  ros::Publisher                         _pubRenderedSpace;
  std::unique_ptr<tf::TransformListener> _listener;
  std::string                            _tfBaseFrame;
  float                                  _dimX;
  float                                  _dimY;
  float                                  _dimZ;
  float                                  _cellSize;
  unsigned int                           _cellsX;
  unsigned int                           _cellsY;
  unsigned int                           _cellsZ;
  bool                                   _virginPush;
  // pcl::PointCloud<pcl::PointXYZRGB>                                                _renderedSpace;
  typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdVecEig3d;

  // for shift down test
  obvious::obfloat _centerspace[3];
  obvious::obfloat _zCoord;

  dynamic_reconfigure::Server<sensor_velodyne3d::VelodyneTsdReconfigureConfig>               _serverReconf;
  dynamic_reconfigure::Server<sensor_velodyne3d::VelodyneTsdReconfigureConfig>::CallbackType _callBackConfig;
  sensor_velodyne3d::VelodyneTsdReconfigureConfig                                            _dynConfig;
};

#endif /* SRC_VELODYNETSD_H_ */
