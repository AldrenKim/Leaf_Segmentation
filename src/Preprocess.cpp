#include "MeshProcessing.h"
#include "PreProcess.h"
#include <pcl/surface/gp3.h>

pcl::PolygonMesh edgeSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::MeshSmoothingLaplacianVTK item;
	pcl::PolygonMesh res = triangulationGreedyProjection(xyzCloud);
	pcl::PolygonMeshConstPtr d(new pcl::PolygonMesh(res));
	item.setInputMesh(d);
	item.process(res);
	return res;
}

void edgeSmoothing1(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::PointCloud<pcl::PointNormal>mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_points(new pcl::search::KdTree<pcl::PointXYZ>);


	mls.setComputeNormals(true);
	mls.setSearchMethod(kdtree_for_points);
	mls.setPolynomialOrder(2);
	mls.setSearchRadius(15);
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::DISTINCT_CLOUD);
	//mls.setUpsamplingStepSize(0.5);
	//mls.setDilationVoxelSize(3);
	//mls.setDilationIterations(2);
	mls.setSqrGaussParam(100);
	//mls.setPointDensity(10);
	mls.setDistinctCloud(xyzCloud);
	mls.setNumberOfThreads(3);
	mls.setInputCloud(xyzCloud);
	mls.process(mls_points);

	if(mls_points.size() > 0)
		pcl::io::savePCDFile("pcdDataSet.pcd", mls_points);
}