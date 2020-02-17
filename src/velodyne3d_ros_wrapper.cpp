/*
 * velodyne3d_ros_wrapper.cpp
 *
 *  Created on: Dec 16, 2019
 *      Author: jasmin
 */

#include "obcore/math/mathbase.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"
#include "obvision/reconstruct/space/SensorVelodyne3D.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/registration/icp/assign/filter/OutOfBoundsFilter3D.h"
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdVecEig3d;

obvious::SensorVelodyne3D*    sensor;
obvious::TsdSpace*            space;
ros::Publisher                pubPulledOutCloud;
ros::Subscriber               subVelodyne;
ros::Publisher                pubRenderedSpace;
ros::Publisher                pubSensorRaycast;
obvious::RayCast3D*           _raycaster;
obvious::OutOfBoundsFilter3D* _filterBounds;

// TEST 1 - nur 3 punkte??
// void pubSensorCast(pcl::PointCloud<pcl::PointXYZ>& cloud)
void pubSensorCast(void)
{
  // doppelt - auch in main, später als member deklarieren in klasse
  unsigned int raysIncl = 16;
  double       inclMin  = obvious::deg2rad(-15.0);
  double       inclRes  = obvious::deg2rad(2.0);
  double       azimRes  = obvious::deg2rad(0.2);

  unsigned int cols   = raysIncl;
  unsigned int rows   = (2 * M_PI) / azimRes;
  unsigned int width  = sensor->getWidth();
  unsigned int height = sensor->getHeight();
  std::cout << __PRETTY_FUNCTION__ << "width: " << width << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "height: " << height << std::endl;

  // unsigned int size = cols * rows * 3;
  unsigned int size = 0;
  std::cout << __PRETTY_FUNCTION__ << "size: " << size << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "cols: " << cols << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "rows: " << rows << std::endl;

  static unsigned int seq = 0;

  std::cout << "Current Transformation: " << std::endl;
  obvious::Matrix T = sensor->getTransformation();
  T.print();

  // _filterBounds->setPose(&T);

  double* coords  = new double[cols * rows * 3];
  double* normals = new double[cols * rows * 3];
  // unsigned int maxSize = space->getXDimension() * space->getYDimension() * space->getZDimension() / 6;
  // std::cout << "maxSize: " << maxSize << std::endl;
  // double*        coords  = new double[maxSize * 3];
  // double*        normals = new double[maxSize * 3];
  // unsigned char* rgb     = new unsigned char[maxSize * 3];

  unsigned int cellsX = space->getXDimension();
  unsigned int cellsY = space->getYDimension();
  unsigned int cellsZ = space->getZDimension();

  std::cout << "aaa " << std::endl;

  // _raycaster->calcCoordsFromCurrentPose(space, sensor, coords, normals, rgb, &size);
  // :calcCoordsFromCurrentPose(TsdSpace* space, Sensor* sensor, double* coords, double* normals, unsigned char* rgb, unsigned int* size)
  _raycaster->calcCoordsFromCurrentPose(space, sensor, coords, normals, NULL, &size);
  std::cout << "bbb " << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "returned by reference size: " << size << std::endl;

  pcl::PointCloud<pcl::PointXYZ> cloud; //--> in velo sub angelegt
  obvious::obfloat               tr[3];
  space->getCentroid(tr);
  std::cout << "ccc " << std::endl;

  for(unsigned int i = 0; i < size; i += 3)
  {
    // std::cout << "ddd " << std::endl;
    pcl::PointXYZ p;
    p.x = coords[i] - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];
    // p.x = -p.x;
    cloud.push_back(p);
  }
  cloud.header.frame_id = "map";
  cloud.header.seq      = seq++;
  pubSensorRaycast.publish(cloud); //--> pub in subscriber velo
  std::cout << "eee " << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "cloud.width = " << cloud.width << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "cloud.height = " << cloud.height << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "cloud.points.size() = " << cloud.points.size() << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;

  const std::string file_name = "/home/jasmin/vagina1.ply";
  pcl::PLYWriter    writer;
  writer.write("/home/jasmin/vagina1.ply", cloud);
}

void renderSpace(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  static unsigned int seq = 0;
  struct Color
  {
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
    Color() : r(0), g(0), b(0) {}
    void red(uint8_t val) { r = val; }
    void blue(uint8_t val) { b = val; }
    uint8_t           r;
    uint8_t           g;
    uint8_t           b;
  };

  obvious::Matrix*   cellCoordsHom = obvious::TsdSpacePartition::getCellCoordsHom();
  obvious::Matrix*   partCoords    = obvious::TsdSpacePartition::getPartitionCoords();
  unsigned int       partSize      = (space->getPartitions())[0][0][0]->getSize();
  stdVecEig3d        centers;
  std::vector<Color> colors;
  obvious::obfloat   tr[3];
  space->getCentroid(tr);
  for(unsigned int pz = 0; pz < space->getPartitionsInZ(); pz++)
  {
    for(unsigned int py = 0; py < space->getPartitionsInY(); py++)
    {
      for(unsigned int px = 0; px < space->getPartitionsInX(); px++)
      {
        obvious::TsdSpacePartition* part = space->getPartitions()[pz][py][px];
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
  cloud.header.frame_id = "map";
  cloud.header.seq      = seq++;
  std::cout << __PRETTY_FUNCTION__ << " publishing " << cloud.size() << " points" << std::endl;
}

void pullOutCloud(void)
{
  unsigned int                  cellsX  = space->getXDimension();
  unsigned int                  cellsY  = space->getYDimension();
  unsigned int                  cellsZ  = space->getZDimension();
  static obvious::obfloat*      coords  = new obvious::obfloat[cellsX * cellsY * cellsZ * 3];
  static obvious::obfloat*      normals = new obvious::obfloat[cellsX * cellsY * cellsZ * 3];
  static unsigned char*         rgb     = new unsigned char[cellsX * cellsY * cellsZ * 3];
  unsigned int                  cnt     = 0;
  static unsigned int           seq     = 0;
  obvious::RayCastAxisAligned3D raycaster;
  raycaster.calcCoords(space, coords, normals, rgb, &cnt);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  obvious::obfloat               tr[3];
  space->getCentroid(tr);

  for(unsigned int i = 0; i < cnt; i += 3)
  {
    pcl::PointXYZ p;
    p.x = coords[i] - tr[0];
    p.y = coords[i + 1] - tr[1];
    p.z = coords[i + 2] - tr[2];
    // p.x = -p.x;
    cloud.push_back(p);
  }
  cloud.header.frame_id = "map";
  cloud.header.seq      = seq++;
  pubPulledOutCloud.publish(cloud);
}

void pubOnePoint(void)
{
  unsigned int        size = sensor->getWidth() * sensor->getHeight();
  std::vector<double> depthData(size, 0.0);
  unsigned int        valid = 0;
  double              depth;

  for(unsigned int i = 0; i < size; i++)
  {
    // // pub one point at 2/2/2 and rest 0/0/0
    // if(i == 0)
    // {
    //   double x = 2.0;
    //   double y = 2.0;
    //   double z = 2.0;
    //   depth    = sqrt(x * x + y * y + z * z);
    // }
    // else
    // {
    //   double x = 0.0;
    //   double y = 0.0;
    //   double z = 0.0;
    //   depth    = sqrt(x * x + y * y + z * z);
    // }

    // depthData[i] = depth;

    // double test = sqrt(2.0 * 2.0 + 2.0 * 2.0 + 2.0 * 2.0);
    // pub torus
    depthData[i] = 1.0;
    valid++;
  }

  std::cout << __PRETTY_FUNCTION__ << "depthData.size() = " << depthData.size() << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points" << std::endl;

  sensor->setRealMeasurementData(depthData.data());
  space->push(sensor);

  std::cout << __PRETTY_FUNCTION__ << " performed push " << std::endl;

  pullOutCloud();
  pcl::PointCloud<pcl::PointXYZRGB> renderedSpaceOnePoint;
  renderSpace(renderedSpaceOnePoint);
  pubRenderedSpace.publish(renderedSpaceOnePoint);
  pubSensorCast();
}

void velodyneCallback(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  // static variable wird nur ein einziges mal initalisiert, also beim erneuten aufruf vom callback nicht wieder überschrieben hier oben
  static bool firstPointcloudIn = false;
  // IM CALLBACK ABFRAGEN OB SPACE SCHON INITIALISIERT IST --> INIT ROUTINE MACHEN
  std::vector<double> depthData(cloud.height * cloud.width, 0.0);
  unsigned int        valid = 0;
  for(unsigned int i = 0; i < cloud.height; i++)
  {
    for(unsigned int j = 0; j < cloud.width; j++)
    {
      const unsigned int idx   = i * cloud.width + j;
      double             x     = double(cloud.points[idx].x);
      double             depth = sqrt(double(cloud.points[idx].x) * double(cloud.points[idx].x) + double(cloud.points[idx].y) * double(cloud.points[idx].y) +
                          double(cloud.points[idx].z) * double(cloud.points[idx].z));

      depthData[idx] = depth; // real measurement data from pointcloud
      // depthData[idx] = 1.0; // artificial data with uniform ray length
      valid++;
    }
  }
  std::cout << __PRETTY_FUNCTION__ << "cloud.height*cloud.width = " << cloud.height * cloud.width << std::endl;
  std::cout << __PRETTY_FUNCTION__ << "depthData.size() = " << depthData.size() << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " pushing " << valid << " valid points" << std::endl;

  sensor->setRealMeasurementData(depthData.data());

  // only one push with first pointcloud
  // if(!firstPointcloudIn)
  // {
  space->push(sensor);
  //   firstPointcloudIn = true;
  // }
  // else
  // {
  //   std::cout << "lalala" << std::endl;
  // }

  // AXIS PARALLEL RAYCASTER
  // pullOutCloud();
  // RENDER SPACE - PHILS METHOD - red blue
  // pcl::PointCloud<pcl::PointXYZRGB> renderedSpace;
  // renderSpace(renderedSpace);
  // pubRenderedSpace.publish(renderedSpace);

  // SENSORVELODYNE3D RAYCASTER
  pubSensorCast();
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "velodyne3d_node");
  ros::NodeHandle nh;
  pubPulledOutCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("pulledOut_cloud", 1);
  pubRenderedSpace  = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("rendered_cloud", 1);
  pubSensorRaycast  = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("sensorRaycast_cloud", 1);

  // instantiate sensor
  unsigned int raysIncl = 16;
  double       inclMin  = obvious::deg2rad(-15.0);
  double       inclRes  = obvious::deg2rad(2.0);
  double       azimRes  = obvious::deg2rad(0.2);
  // sensor                = new obvious::SensorVelodyne3D(raysIncl, inclMin, inclRes, azimRes, 100.0, 0.0, 20.0);
  sensor = new obvious::SensorVelodyne3D(raysIncl, inclMin, inclRes, azimRes);
  // instantiate tsd space
  const double voxelSize = 0.01;

  space = new obvious::TsdSpace(voxelSize, obvious::LAYOUT_8x8x8, obvious::LAYOUT_512x512x512);
  space->setMaxTruncation(3.0 * voxelSize);

  // transform sensor
  // 1 translation
  obvious::obfloat translation[3];
  space->getCentroid(translation);
  // 2 rotation
  double          transform[16] = {1, 0, 0, translation[0], 0, 1, 0, translation[1], 0, 0, 1, translation[2], 0, 0, 0, 1};
  obvious::Matrix MsensorTransform(4, 4);
  MsensorTransform.setIdentity();
  MsensorTransform.setData(transform);
  sensor->setTransformation(MsensorTransform);
  cout << "Initial Pose" << endl;
  obvious::Matrix Tmp = sensor->getTransformation();
  Tmp.print();

  // for velodyne_unpack.bag
  // subVelodyne = nh.subscribe("velodyne_points", 1, velodyneCallback);

  // for handstand or boxes bagfile
  subVelodyne = nh.subscribe("/puck_rear/velodyne_points", 1, velodyneCallback);
  _raycaster  = new obvious::RayCast3D();

  // Test - published einen Torus
  // pubOnePoint();
  ros::spin();

  // delete space;
  // delete _raycaster;
  // delete sensor;
  // delete coords;
  // delete normals;
}
