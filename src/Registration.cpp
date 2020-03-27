/*
 * Registration.cpp
 *
 *  Created on: Jan 8, 2018
 *      Author: phil
 */

#include "Registration.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

Registration::Registration()
{
  // TODO Auto-generated constructor stub
}

Registration::~Registration()
{
  // TODO Auto-generated destructor stub
}

const Eigen::Matrix4f Registration::alignPcl(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelSrc, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneSrc,
                                             const float eps, const unsigned int iterationsMax, const float distMax, const bool downsample,
                                             const float leafSize, pcl::PointCloud<pcl::PointNormal>::Ptr aligned)
{
  // qDebug() << __PRETTY_FUNCTION__ << " starting registration with desired eps " << eps << " iterations " << iterationsMax << " distTrhesh " << distMax;
  // if(downsample)
  //   qDebug() << "Downsampling active with a leaf size of " << leafSize;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB>       grid;
  if(downsample)
  {
    grid.setLeafSize(leafSize, leafSize, leafSize);
    grid.setInputCloud(modelSrc);
    grid.filter(*model);

    grid.setInputCloud(sceneSrc);
    grid.filter(*scene);
  }
  else
  {
    model = modelSrc;
    scene = sceneSrc;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr modelNorms(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sceneNorms(new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(model);
  norm_est.compute(*modelNorms);
  pcl::copyPointCloud(*model, *modelNorms);

  norm_est.setInputCloud(scene);
  norm_est.compute(*sceneNorms);
  pcl::copyPointCloud(*scene, *sceneNorms);

  MyPointRepresentation point_representation; // todo: dont know why we need this...
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon(eps);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(distMax);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(modelNorms);
  reg.setInputTarget(sceneNorms);

  Eigen::Matrix4f Ti             = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev           = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f targetToSource = Eigen::Matrix4f::Identity();
  reg.setMaximumIterations(iterationsMax);

  reg.align(*aligned);
  // reg.get
  // qDebug() << __PRETTY_FUNCTION__ << " fitness score " << reg.getEuclideanFitnessEpsilon();

  Ti = reg.getFinalTransformation();
  return Ti.inverse();
}

const Eigen::Matrix4f Registration::alignPcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr modelSrc, const pcl::PointCloud<pcl::PointXYZ>::Ptr sceneSrc,
                                             const float eps, const unsigned int iterationsMax, const float distMax, const bool downsample,
                                             const float leafSize, pcl::PointCloud<pcl::PointNormal>::Ptr aligned)
{
  // qDebug() << __PRETTY_FUNCTION__ << " starting registration with desired eps " << eps << " iterations " << iterationsMax << " distTrhesh " << distMax;
  // if(downsample)
  //   qDebug() << "Downsampling active with a leaf size of " << leafSize;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ>       grid;
  if(downsample)
  {
    grid.setLeafSize(leafSize, leafSize, leafSize);
    grid.setInputCloud(modelSrc);
    grid.filter(*model);

    grid.setInputCloud(sceneSrc);
    grid.filter(*scene);
  }
  else
  {
    model = modelSrc;
    scene = sceneSrc;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr modelNorms(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sceneNorms(new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(model);
  norm_est.compute(*modelNorms);
  pcl::copyPointCloud(*model, *modelNorms);

  norm_est.setInputCloud(scene);
  norm_est.compute(*sceneNorms);
  pcl::copyPointCloud(*scene, *sceneNorms);

  MyPointRepresentation point_representation; // todo: dont know why we need this...
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon(eps);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(distMax);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(modelNorms);
  reg.setInputTarget(sceneNorms);

  Eigen::Matrix4f Ti             = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev           = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f targetToSource = Eigen::Matrix4f::Identity();
  reg.setMaximumIterations(iterationsMax);

  reg.align(*aligned);
  // qDebug() << __PRETTY_FUNCTION__ << " fitness score " << reg.getEuclideanFitnessEpsilon();

  Ti = reg.getFinalTransformation();
  if(reg.hasConverged())
    return Ti.inverse();
  else
    return Eigen::Matrix4f::Identity();
}

const Eigen::Matrix4f Registration::alignPcl(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr modelSrc,
                                             const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sceneSrc, const float eps, const unsigned int iterationsMax,
                                             const float distMax, const bool downsample, const float leafSize,
                                             pcl::PointCloud<pcl::PointNormal>::Ptr aligned, bool& hasConverged)
{
  // qDebug() << __PRETTY_FUNCTION__ << " starting registration with desired eps " << eps << " iterations " << iterationsMax << " distTrhesh " << distMax;
  // if(downsample)
  //   qDebug() << "Downsampling active with a leaf size of " << leafSize;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::VoxelGrid<pcl::PointXYZRGBNormal>       grid;
  if(downsample)
  {
    grid.setLeafSize(leafSize, leafSize, leafSize);
    grid.setInputCloud(modelSrc);
    grid.filter(*model);

    grid.setInputCloud(sceneSrc);
    grid.filter(*scene);
  }
  else
  {
    model = modelSrc;
    scene = sceneSrc;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr modelNorms(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr sceneNorms(new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(model);
  norm_est.compute(*modelNorms);
  pcl::copyPointCloud(*model, *modelNorms);

  norm_est.setInputCloud(scene);
  norm_est.compute(*sceneNorms);
  pcl::copyPointCloud(*scene, *sceneNorms);

  MyPointRepresentation point_representation; // todo: dont know why we need this...
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon(eps);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(distMax);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(modelNorms);
  reg.setInputTarget(sceneNorms);

  Eigen::Matrix4f Ti             = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f prev           = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f targetToSource = Eigen::Matrix4f::Identity();
  reg.setMaximumIterations(iterationsMax);

  reg.align(*aligned);
  // reg.get
  //   qDebug() << __PRETTY_FUNCTION__ << " fitness score " << reg.getEuclideanFitnessEpsilon();

  Ti           = reg.getFinalTransformation();
  hasConverged = reg.hasConverged();
  return Ti.inverse();
}
// const Eigen::Affine3f& Registration::alignPclStep(const pcl::PointCloud<pcl::PointXYZ>::Ptr model, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
//{
//
//}
