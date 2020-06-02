 /*
 * Registration.h
 *
 *  Created on: Jan 8, 2018
 *      Author: phil
 */

#ifndef OHM_DEEP_RECON_SRC_REGISTRATION_H_
#define OHM_DEEP_RECON_SRC_REGISTRATION_H_

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }
  virtual ~MyPointRepresentation(void){}
  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

class Registration
{
public:
  Registration();
  virtual ~Registration();
  const Eigen::Matrix4f alignPcl(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, const float eps,
      const unsigned int iterationsMax, const float distMax, const bool downsample, const float leafSize, pcl::PointCloud<pcl::PointNormal>::Ptr aligned);
  const Eigen::Matrix4f alignPcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr model, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene, const float eps,
        const unsigned int iterationsMax, const float distMax, const bool downsample, const float leafSize, pcl::PointCloud<pcl::PointNormal>::Ptr aligned);
  const Eigen::Matrix4f alignPcl(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelSrc, const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sceneSrc, const float eps,
        const unsigned int iterationsMax, const float distMax, const bool downsample, const float leafSize, pcl::PointCloud<pcl::PointNormal>::Ptr aligned, bool& hasConverged);
};

#endif /* OHM_DEEP_RECON_SRC_REGISTRATION_H_ */
