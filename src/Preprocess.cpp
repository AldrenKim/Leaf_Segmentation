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