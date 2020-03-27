/*
 * VelodyneTsd.h
 *
 *  Created on: Jan 7, 2020
 *      Author: jasmin
 */

#ifndef SRC_VELODYNETSD_H_
#define SRC_VELODYNETSD_H_

#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/SensorVelodyne3D.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include "sensor_velodyne3d/VelodyneTsdReconfigureConfig.h"
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class VelodyneTsd
{
public:
  VelodyneTsd(const float dimX, const float dimY, const float dimZ, const float cellSize);
  virtual ~VelodyneTsd();
  void callbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void pubSensorRaycast(const pcl::PointCloud<pcl::PointXYZ>& currentLaserSceneData);
  void matchScanToSpace(const pcl::PointCloud<pcl::PointXYZ>& currentLaserSceneData, const pcl::PointCloud<pcl::PointXYZ>& lastRaycastModelData);

private:
  void init(const pcl::PointCloud<pcl::PointXYZ>& cloud);
  void callbackDynReconf(sensor_velodyne3d::VelodyneTsdReconfigureConfig& config, uint32_t level);
  std::unique_ptr<obvious::TsdSpace>         _space;
  std::unique_ptr<obvious::SensorVelodyne3D> _sensor;

  obvious::RayCast3D*                    _raycaster;
  ros::NodeHandle                        _nh;
  ros::Subscriber                        _subPointCloud;
  ros::Publisher                         _pubSensorRaycast;
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

  dynamic_reconfigure::Server<sensor_velodyne3d::VelodyneTsdReconfigureConfig>               _serverReconf;
  dynamic_reconfigure::Server<sensor_velodyne3d::VelodyneTsdReconfigureConfig>::CallbackType _callBackConfig;
  sensor_velodyne3d::VelodyneTsdReconfigureConfig                                            _dynConfig;
};

#endif /* SRC_VELODYNETSD_H_ */
