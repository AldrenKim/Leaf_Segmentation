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
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/2d/morphology.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

typedef pcl::PointXYZ Point;

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

//helpers
void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d& data);
void visualizeCurve(ON_NurbsCurve & curve,
	                ON_NurbsSurface & surface,
					boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

pcl::PolygonMesh edgeSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PolygonMesh poisson_recon_MLS(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PolygonMesh poisson_recon_MLS_Pass_NE(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PolygonMesh poisson_recon_MLS_Pass(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
void edgeSmoothing1(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PointCloud<pcl::PointNormal> smoothingMLS(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PolygonMesh bsplineFitting(pcl::PointCloud<Point>::Ptr xyzCloud); // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

pcl::PolygonMesh edgeSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
void edgeSmoothing1(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr morph(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleFile(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);

void ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);

void addSupervoxelConnectionsToViewer(PointT& supervoxel_center, PointCloudT& adjacent_supervoxel_centers, std::string supervoxel_name, pcl::visualization::PCLVisualizer::Ptr& viewer);

void supervoxels_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);