#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

pcl::PolygonMesh edgeSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
void edgeSmoothing1(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);