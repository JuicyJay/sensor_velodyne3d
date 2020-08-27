/*
 * VelodyneTsd.cpp
 *
 *  Created on: Jan 7, 2020
 *      Author: jasmin
 */

#include "VelodyneTsd.h"
#include "Registration.h"
#include "obcore/math/mathbase.h"
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

VelodyneTsd::VelodyneTsd(const float dimX, const float dimY, const float dimZ, const float cellSize)
    : _dimX(dimX), _dimY(dimY), _dimZ(dimZ), _cellSize(cellSize), _listener(std::make_unique<tf::TransformListener>()), _cellsX(0), _cellsY(0), _cellsZ(0)
{
  _subPointCloud = _nh.subscribe("puck_rear/velodyne_points", 1, &VelodyneTsd::callbackPointCloud, this);
  // _tfBaseFrame      = "base_link";
  _tfBaseFrame      = "map"; // changed to map for artificial box data from cloud_factory
  _raycaster        = new obvious::RayCast3D();
  _pubSensorRaycast = _nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("sensorRaycast", 1);
  _pubRenderedSpace = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("rendered_cloud", 1);
  _virginPush       = false;
  _callBackConfig   = boost::bind(&VelodyneTsd::callbackDynReconf, this, _1, _2);
  _serverReconf.setCallback(_callBackConfig);
}

VelodyneTsd::~VelodyneTsd() { delete _raycaster; }

void VelodyneTsd::init(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  std::cout << __PRETTY_FUNCTION__ << " hey " << std::endl;
  if(_sensor || _space)
    return;

  unsigned int cellsX = static_cast<unsigned int>(std::round(_dimX / _cellSize));
  unsigned int cellsY = static_cast<unsigned int>(std::round(_dimY / _cellSize));
  unsigned int cellsZ = static_cast<unsigned int>(std::round(_dimZ / _cellSize));

  std::cout << __PRETTY_FUNCTION__ << "cellsX " << cellsX << " cellsY " << cellsY << " cellsZ " << cellsZ << std::endl;
  // wtf is this supposed to do
  const unsigned int xFac = cellsX / 64; // todo: rename fac with FUCK...Obviously
  const unsigned int yFac = cellsY / 64;
  const unsigned int zFac = cellsZ / 64;

  cellsX = xFac * 64;
  cellsY = yFac * 64;
  cellsZ = zFac * 64;

  std::cout << __PRETTY_FUNCTION__ << "cellsX " << cellsX << " cellsY " << cellsY << " cellsZ " << cellsZ << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "_cellSize =  " << _cellSize << std::endl;

  _cellsX = cellsX;
  _cellsY = cellsY;
  _cellsZ = cellsZ;

  _space = std::make_unique<obvious::TsdSpace>(_cellSize, obvious::LAYOUT_64x64x64, cellsX, cellsY, cellsZ);
  _space->setMaxTruncation(3.0 * _cellSize);
  // initial pose of sensor in middle of space --> transformed to last pose in callbackPointCloud
  double tr[3];
  _space->getCentroid(tr);
  double          tf[16] = {1, 0, 0, tr[0], 0, 1, 0, tr[1], 0, 0, 1, tr[2], 0, 0, 0, 1};
  obvious::Matrix Tinit(4, 4);
  Tinit.setIdentity();
  Tinit.setData(tf);

  std::cout << __PRETTY_FUNCTION__ << "aa" << std::endl;
  // switch here if you want to use SensorVelodyne3D // SensorPolar3DBase
  // SensorVelodyne3D
  unsigned int raysIncl = 16;
  double       inclMin  = obvious::deg2rad(-15.0);
  double       inclRes  = obvious::deg2rad(2.0);
  double       azimRes  = obvious::deg2rad(0.2);
  // set higher for easier debugging
  // double azimRes = obvious::deg2rad(3.6);

  _sensor = std::make_unique<obvious::SensorVelodyne3D>(raysIncl, inclMin, inclRes, azimRes, 100.0, 0.0, 20.0);

  // SensorPolar3DBase
  // double inclMin = obvious::deg2rad(-15.0);
  // double inclMax = obvious::deg2rad(15.0);
  // double inclRes = obvious::deg2rad(2.0);
  // double azimMin = obvious::deg2rad(0.0);
  // double azimMax = obvious::deg2rad(360.0);
  // double azimRes = obvious::deg2rad(0.2);
  // std::cout << __PRETTY_FUNCTION__ << "1234455" << std::endl;
  // _sensor = std::make_unique<obvious::SensorPolar3DBase>(inclMin, inclMax, inclRes, azimMin, azimMax, azimRes, 100.0, 0.0, 20.0);

  std::cout << __PRETTY_FUNCTION__ << "aabb" << std::endl;

  _sensor->setTransformation(Tinit);
  std::cout << "Initial Pose - sensor set to the middle of tsd space in init: " << std::endl;
  obvious::Matrix Tmp = _sensor->getTransformation();
  Tmp.print();

  // for shift down test
  // _space->getCentroid(_centerspace);
  // _zCoord = _centerspace[2];
}
void VelodyneTsd::callbackDynReconf(sensor_velodyne3d::VelodyneTsdReconfigureConfig& config, uint32_t level)
{
  std::cout << __PRETTY_FUNCTION__ << " hey gorgeous, this is the dynreconf callback." << std::endl;
  _dynConfig = config;
}

// raycast liefert model aus dem aktuellen blinkwinkel der sensorpose aus dem letzten und allen vorausgegangenen pushs
// scene wird mit currentLaserData weitergereicht
bool VelodyneTsd::pubSensorRaycast()
{
  static obvious::obfloat* coords  = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static obvious::obfloat* normals = new obvious::obfloat[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned char*    rgb     = new unsigned char[_cellsX * _cellsY * _cellsZ * 3];
  static unsigned int      seq     = 0;

  std::cout << __PRETTY_FUNCTION__ << "Current Transformation: " << std::endl;
  obvious::Matrix T = _sensor->getTransformation();
  T.print();

  // _filterBounds->setPose(&T);
  unsigned int width  = _sensor->getWidth();
  unsigned int height = _sensor->getHeight();

  // double*      coords  = new double[width * height * 3];
  // double*      normals = new double[width * height * 3];
  unsigned int size = 0;

  _raycaster->calcCoordsFromCurrentPose(_space.get(), _sensor.get(), coords, normals, rgb, &size);

  pcl::PointCloud<pcl::PointXYZRGBNormal> lastRaycastModelData;
  obvious::obfloat                        tr[3];
  _space->getCentroid(tr);

  for(unsigned int i = 0; i < size; i += 3)
  {
    pcl::PointXYZRGBNormal p;

    // x y z
    // p.x = coords[i] - tr[0];
    // p.y = coords[i + 1] - tr[1];
    // p.z = coords[i + 2] - tr[2];
    // p.x = -p.x;

    // x z y
    p.x = coords[i] - tr[0];
    p.z = coords[i + 1] - tr[1];
    p.y = coords[i + 2] - tr[2];
    p.x = -p.x; /// WAS WIRD HIER GEDREHT??

    p.normal_x = normals[i] - tr[0];
    p.normal_y = normals[i + 1] - tr[1];
    p.normal_z = normals[i + 2] - tr[2];

    p.r = rgb[i];
    p.g = rgb[i + 1];
    p.b = rgb[i + 2];

    lastRaycastModelData.push_back(p);
  }
  lastRaycastModelData.header.frame_id = "map";
  lastRaycastModelData.header.seq      = seq++;
  _pubSensorRaycast.publish(lastRaycastModelData);
  return true;

  // ALLES IM CALLBACK TRIGGERN DU DEPP
  // this->matchScanToSpace(currentLaserSceneData, lastRaycastModelData);

  // RENDER SPACE - PHILS METHOD - red blue
  // this->renderSpace(_renderedSpace);
}

// MODEL = raycast from current point of view of sensor --in first iteration: data from virgin push
// SCENE = most recent sensor data
//  const Eigen::Matrix4f alignPcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr model, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene, const float eps,
// const unsigned int iterationsMax, const float distMax, const bool downsample, const float leafSize, pcl::PointCloud<pcl::PointNormal>::Ptr aligned);

// void VelodyneTsd::matchScanToSpace(const pcl::PointCloud<pcl::PointXYZ>& currentLaserSceneData, const pcl::PointCloud<pcl::PointXYZ>&
// lastRaycastModelData)
// {
//   std::cout << __PRETTY_FUNCTION__ << "huhu" << std::endl;
//   Registration reg;

//   // Registration params
//   const float        eps           = _dynConfig.pcl_icp_eps;
//   const unsigned int iterationsMax = _dynConfig.pcl_icp_iter;
//   const float        distMax       = _dynConfig.pcl_icp_dist;
//   const bool         downsample    = _dynConfig.pcl_icp_dwnspl;
//   const float        leafSize      = _dynConfig.pcl_icp_leafs;

//   pcl::PointCloud<pcl::PointNormal>::Ptr    aligned(new pcl::PointCloud<pcl::PointNormal>());
//   const pcl::PointCloud<pcl::PointXYZ>::Ptr modelPtr(new pcl::PointCloud<pcl::PointXYZ>());
//   *modelPtr = lastRaycastModelData;
//   const pcl::PointCloud<pcl::PointXYZ>::Ptr scenePtr(new pcl::PointCloud<pcl::PointXYZ>());
//   *scenePtr = currentLaserSceneData;

//   const Eigen::Matrix4f regResultTransformMatrix = reg.alignPcl(modelPtr, scenePtr, eps, iterationsMax, distMax, downsample, leafSize, aligned);
//   std::cout << "registration result regResultTransformMatrix = \n" << regResultTransformMatrix << std::endl;
// }

void VelodyneTsd::renderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  static unsigned int seq = 0;
  struct Color
  {
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
    Color() : r(0), g(0), b(0) {}
    void    red(uint8_t val) { r = val; }
    void    blue(uint8_t val) { b = val; }
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  obvious::Matrix*   cellCoordsHom = obvious::TsdSpacePartition::getCellCoordsHom();
  obvious::Matrix*   partCoords    = obvious::TsdSpacePartition::getPartitionCoords();
  unsigned int       partSize      = (_space->getPartitions())[0][0][0]->getSize();
  stdVecEig3d        centers;
  std::vector<Color> colors;
  obvious::obfloat   tr[3];
  _space->getCentroid(tr);
  for(unsigned int pz = 0; pz < _space->getPartitionsInZ(); pz++)
  {
    for(unsigned int py = 0; py < _space->getPartitionsInY(); py++)
    {
      for(unsigned int px = 0; px < _space->getPartitionsInX(); px++)
      {
        obvious::TsdSpacePartition* part = _space->getPartitions()[pz][py][px];
        if(part->isInitialized() && !part->isEmpty())
        {
          obvious::obfloat t[3];
          part->getCellCoordsOffset(t);
          for(unsigned int c = 0; c < partSize; c++)
          {
            Eigen::Vector3d center;
            center(0) = (*cellCoordsHom)(c, 0) + t[0];
            center(1) = (*cellCoordsHom)(c, 1) + t[1];
            center(2) = (*cellCoordsHom)(c, 2) + t[2];
            // if(center(1) > _space->getMaxY() / 2.0)
            //   continue;
            obvious::obfloat tsd = part->getTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2));
            if((isnan(tsd)) || (tsd > std::abs(1.1)))
              continue;

            Color tsdColor; //(0, 0, 0);
            if(tsd < 0.0)   // red
              tsdColor.red(static_cast<unsigned char>(-1.0 * tsd * 255.0));
            else
              tsdColor.blue(static_cast<unsigned char>(tsd * 255.0));

            centers.push_back(center);
            colors.push_back(tsdColor);
          }
        }
      }
    }
  }
  if((centers.size() == colors.size()) && (centers.size() != 0))
  {
    //_gui->drawGlyphs(colors, centers, space.getVoxelSize());
    for(unsigned int i = 0; i < centers.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x = centers[i].x() - tr[0];
      p.y = centers[i].y() - tr[1];
      p.z = centers[i].z() - tr[2];
      p.r = colors[i].r;
      p.g = colors[i].g;
      p.b = colors[i].b;
      cloud.push_back(p);
    }
  }
  else
    std::cout << __PRETTY_FUNCTION__ << "nuttingham found " << centers.size() << " " << colors.size() << std::endl;
  cloud.header.frame_id = _tfBaseFrame;
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;

  // static unsigned int seq = 0;
  // struct Color
  // {
  //   Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
  //   Color() : r(0), g(0), b(0) {}
  //   void    red(uint8_t val) { r = val; }
  //   void    blue(uint8_t val) { b = val; }
  //   uint8_t r;
  //   uint8_t g;
  //   uint8_t b;
  // };

  // obvious::Matrix*   cellCoordsHom = obvious::TsdSpacePartition::getCellCoordsHom();
  // obvious::Matrix*   partCoords    = obvious::TsdSpacePartition::getPartitionCoords();
  // unsigned int       partSize      = (_space->getPartitions())[0][0][0]->getSize();
  // stdVecEig3d        centers;
  // std::vector<Color> colors;
  // obvious::obfloat   tr[3];
  // _space->getCentroid(tr);
  // for(unsigned int pz = 0; pz < _space->getPartitionsInZ(); pz++)
  // {
  //   for(unsigned int py = 0; py < _space->getPartitionsInY(); py++)
  //   {
  //     for(unsigned int px = 0; px < _space->getPartitionsInX(); px++)
  //     {
  //       obvious::TsdSpacePartition* part = _space->getPartitions()[pz][py][px];
  //       if(part->isInitialized() && !part->isEmpty())
  //       {
  //         obvious::obfloat t[3];
  //         part->getCellCoordsOffset(t);
  //         for(unsigned int c = 0; c < partSize; c++)
  //         {
  //           Eigen::Vector3d center;
  //           center(0) = (*cellCoordsHom)(c, 0) + t[0];
  //           center(1) = (*cellCoordsHom)(c, 1) + t[1];
  //           center(2) = (*cellCoordsHom)(c, 2) + t[2];

  //           obvious::obfloat tsd = part->getTsd((*partCoords)(c, 0), (*partCoords)(c, 1), (*partCoords)(c, 2));
  //           if((isnan(tsd)) || (tsd > std::abs(1.1)))
  //             continue;

  //           Color tsdColor; //(0, 0, 0);
  //           if(tsd < 0.0)   // red
  //             tsdColor.red(static_cast<unsigned char>(-1.0 * tsd * 255.0));
  //           else
  //             tsdColor.blue(static_cast<unsigned char>(tsd * 255.0));

  //           centers.push_back(center);
  //           colors.push_back(tsdColor);
  //         }
  //       }
  //     }
  //   }
  // }
  // if((centers.size() == colors.size()) && (centers.size() != 0))
  // {
  //   for(unsigned int i = 0; i < centers.size(); i++)
  //   {
  //     pcl::PointXYZRGB p;
  //     p.x = centers[i].x() - tr[0];
  //     p.y = centers[i].y() - tr[1];
  //     p.z = centers[i].z() - tr[2];
  //     p.r = colors[i].r;
  //     p.g = colors[i].g;
  //     p.b = colors[i].b;
  //     cloud.push_back(p);
  //   }
  // }
  // else
  //   std::cout << __PRETTY_FUNCTION__ << "nuttingham found " << centers.size() << " " << colors.size() << std::endl;
  // cloud.header.frame_id = "map";
  // cloud.header.seq      = seq++;
  // std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
  // _pubRenderedSpace.publish(_renderedSpace);
}

void VelodyneTsd::callbackPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if(!_sensor || !_space)
    this->init(cloud);

  ros::Time            elapsedTimer = ros::Time::now();
  tf::StampedTransform tfSensor;

  // look up transform from map to _tfBaseFrame = base_link for my sensor to transform sensor!
  try
  {
    _listener->lookupTransform(_tfBaseFrame, "map", ros::Time(0), tfSensor);
  }
  catch(const tf::TransformException& e)
  {
    std::cout << __PRETTY_FUNCTION__ << "e: " << e.what() << std::endl;
    return;
  }

  // nur 1 Push für Registrierung -- auskomemntiert jetzt
  //   if(!_virginPush)
  //   {
  //     std::cout << __PRETTY_FUNCTION__ << "virgin push, first point cloud in" << std::endl;

  //     tf::Quaternion quat  = tfSensor.getRotation();
  //     double         roll  = 0.0;
  //     double         pitch = 0.0;
  //     double         yaw   = 0.0;
  //     tf::Matrix3x3  RotMat(quat);
  //     RotMat.getRPY(roll, pitch, yaw);
  //     std::cout << __PRETTY_FUNCTION__ << "RPY " << roll << " " << pitch << " " << yaw << " " << std::endl;

  //     obvious::Matrix TransMat(4, 4);
  //     TransMat.setIdentity();
  //     obvious::obfloat center[3];
  //     _space->getCentroid(center);
  //     tf::Vector3 tfVec = tfSensor.getOrigin();

  //     std::cout << "tfVec.getX() = " << tfVec.getX() << " tfVec.getY() = " << tfVec.getY() << " tfVec.getZ() = " << tfVec.getX() << std::endl;
  //     std::cout << "tfSensor.getOrigin().getX() = " << tfSensor.getOrigin().getX() << " tfSensor.getOrigin().getY() = " << tfSensor.getOrigin().getY()
  //               << " tfSensor.getOrigin().getZ() = " << tfSensor.getOrigin().getZ() << std::endl;
  //     TransMat = obvious::MatrixFactory::TransformationMatrix44(yaw, pitch, roll, center[0] + tfVec.getX(), center[1] + tfSensor.getOrigin().getY(),
  //                                                               center[2] + tfSensor.getOrigin().getZ());
  //     // TRANSFORM SENSOR
  //     _sensor->setTransformation(TransMat);

  //     std::cout << __PRETTY_FUNCTION__ << "Current Transformation after sensor transform with tf: " << std::endl;
  //     obvious::Matrix TransCheck = _sensor->getTransformation();
  //     TransCheck.print();

  //     // extract data from pointcloud and write it to depthData
  //     std::vector<double> depthData(cloud.height * cloud.width, 0.0);
  //     bool*               mask = new bool[cloud.height * cloud.width];
  //     // std::vector<bool> mask(cloud.height * cloud.width, false);

  //     std::cout << "VelodyneTsd cloud.height = " << cloud.height << " , cloud.width = " << cloud.width << std::endl;

  //     // WHAT HAPPENS IF I CHANGE HEIGHT AND WIDTH HERE
  //     // unsigned int valid = 0;
  //     // for(unsigned int i = 0; i < cloud.height; i++)
  //     // {
  //     //   for(unsigned int j = 0; j < cloud.width; j++)
  //     //   {
  //     //     const unsigned int idx = i * cloud.width + j;
  //     //     Eigen::Vector3f    point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);

  //     //     double abs = static_cast<double>(point.norm());
  //     //     if(abs > 0.0)
  //     //     {
  //     //       depthData[idx] = abs;
  //     //       std::cout << "depthData[idx] = " << depthData[idx] << " at idx = " << idx << std::endl;
  //     //       // depthData[idx] = 1.0; // artificial data with uniform ray length -- wird ne Kugel
  //     //       mask[idx] = true;
  //     //       valid++;
  //     //     }
  //     //   }
  //     // }

  //     unsigned int valid = 0;
  //     for(unsigned int i = 0; i < cloud.height; i++)
  //     {
  //       for(unsigned int j = 0; j < cloud.width; j++)
  //       {
  //         const unsigned int idx = i * cloud.width + j;
  //         Eigen::Vector3f    point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);

  //         double abs = static_cast<double>(point.norm());
  //         if(abs > 0.0)
  //         {
  //           depthData[idx] = abs;
  //           // depthData[idx] = 1.0; // artificial data with uniform ray length -- wird ne Kugel
  //           mask[idx] = true;
  //           valid++;
  //         }
  //       }
  //     }
  //     if(!valid)
  //     {
  //       std::cout << __PRETTY_FUNCTION__ << " no valid points in data " << std::endl;
  //       return;
  //     }
  //     std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points" << std::endl;
  //     _sensor->setRealMeasurementData(depthData.data());
  //     std::cout << __PRETTY_FUNCTION__ << "dddd" << std::endl;
  //     _sensor->setRealMeasurementMask(mask);
  //     std::cout << __PRETTY_FUNCTION__ << "eee" << std::endl;

  //     _space->push(_sensor.get());

  //     std::cout << __PRETTY_FUNCTION__ << "fff" << std::endl;
  //     _virginPush = true;
  //     delete mask;
  //   }
  //   else
  //   {
  //     // NACH UNTEN VERSCHIEBEN Test
  //     // _zCoord -= 0.3;
  //     // if(_zCoord < 0.0)
  //     // {
  //     //   return;
  //     // }
  //     // obvious::Matrix TransMatShift(4, 4);
  //     // TransMatShift.setIdentity();

  //     // TransMatShift = obvious::MatrixFactory::TransformationMatrix44(0.0, 0.0, 0.0, _centerspace[0], _centerspace[1], _zCoord);
  //     // // TRANSFORM SENSOR
  //     // _sensor->setTransformation(TransMatShift);

  //     // std::cout << __PRETTY_FUNCTION__ << "Current Transformation after downshift: " << std::endl;
  //     // obvious::Matrix TransCheckShift = _sensor->getTransformation();
  //     // TransCheckShift.print();

  //     // std::cout << __PRETTY_FUNCTION__ << "virgin push over, sensor shifted down by z=-0.3m, continue with raycast" << std::endl;
  //     // // CONTINUE WITH RAYCAST HERE
  //     // this->pubSensorRaycast(cloud);

  //     // REGISTRATION TEST
  //     std::cout << __PRETTY_FUNCTION__ << "virgin push over, continue with registration" << std::endl;
  //     // CONTINUE WITH RAYCAST HERE
  //     this->pubSensorRaycast(cloud);
  //   }
  // }

  tf::Quaternion quat  = tfSensor.getRotation();
  double         roll  = 0.0;
  double         pitch = 0.0;
  double         yaw   = 0.0;
  tf::Matrix3x3  RotMat(quat);
  RotMat.getRPY(roll, pitch, yaw);
  std::cout << __PRETTY_FUNCTION__ << "RPY " << roll << " " << pitch << " " << yaw << " " << std::endl;

  obvious::Matrix TransMat(4, 4);
  TransMat.setIdentity();
  obvious::obfloat center[3];
  _space->getCentroid(center);
  tf::Vector3 tfVec = tfSensor.getOrigin();

  std::cout << "tfVec.getX() = " << tfVec.getX() << " tfVec.getY() = " << tfVec.getY() << " tfVec.getZ() = " << tfVec.getX() << std::endl;
  std::cout << "tfSensor.getOrigin().getX() = " << tfSensor.getOrigin().getX() << " tfSensor.getOrigin().getY() = " << tfSensor.getOrigin().getY()
            << " tfSensor.getOrigin().getZ() = " << tfSensor.getOrigin().getZ() << std::endl;
  TransMat = obvious::MatrixFactory::TransformationMatrix44(yaw, pitch, roll, center[0] + tfVec.getX(), center[1] + tfSensor.getOrigin().getY(),
                                                            center[2] + tfSensor.getOrigin().getZ());
  // TRANSFORM SENSOR
  _sensor->setTransformation(TransMat);

  std::cout << __PRETTY_FUNCTION__ << "Current Transformation after sensor transform with tf: " << std::endl;
  obvious::Matrix TransCheck = _sensor->getTransformation();
  TransCheck.print();

  // extract data from pointcloud and write it to depthData
  std::vector<double> depthData(cloud.height * cloud.width, 0.0);
  bool*               mask = new bool[cloud.height * cloud.width];

  std::cout << "VelodyneTsd cloud.height = " << cloud.height << " , cloud.width = " << cloud.width << std::endl;

  unsigned int valid = 0;
  for(unsigned int i = 0; i < cloud.height; i++)
  {
    for(unsigned int j = 0; j < cloud.width; j++)
    {
      const unsigned int idx = i * cloud.width + j;
      Eigen::Vector3f    point(cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z);

      double abs = static_cast<double>(point.norm());
      if(abs > 0.0)
      {
        depthData[idx] = abs;
        // depthData[idx] = 1.0; // artificial data with uniform ray length -- wird ne Kugel
        mask[idx] = true;
        valid++;
      }
      else
        mask[idx] = false;
    }
  }
  if(!valid)
  {
    std::cout << __PRETTY_FUNCTION__ << " no valid points in data " << std::endl;
    return;
  }
  std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points" << std::endl;
  _sensor->setRealMeasurementData(depthData.data());
  _sensor->setRealMeasurementMask(mask);

  _space->push(_sensor.get());
  _virginPush = true;
  delete mask;

  // ALLES ÜBER DEN CALLBACK UND NEUE PCLOUD! DAS WAR ANDERS!
  pcl::PointCloud<pcl::PointXYZRGB> renderedSpace;
  this->renderSpace(renderedSpace);
  _pubRenderedSpace.publish(renderedSpace);
  this->pubSensorRaycast();
}

// vll hilft dieser aufbau mit den rviz updates ????????????????
// if(!valid)
// {
//   std::cout << __PRETTY_FUNCTION__ << " no valid points in data " << std::endl;
//   return;
// }
// std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points" << std::endl;
// _sensor->setRealMeasurementData(depthData.data());
// _sensor->setRealMeasurementMask(mask);
// delete mask;
// _space->push(_sensor.get());
// this->pubSumCloud();
// pcl::PointCloud<pcl::PointXYZRGB> renderedSpace;
// this->renderSpace(renderedSpace);

// _pubRenderedSpace.publish(renderedSpace);
//