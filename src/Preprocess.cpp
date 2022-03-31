#include "MeshProcessing.h"
#include "PreProcess.h"
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>

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
	// For VOXEL_Grid_Dilation
	//mls.setDilationVoxelSize(3);
	//mls.setDilationIterations(2);
	mls.setSqrGaussParam(100);
	//For Random_Uniform_Density
	//mls.setPointDensity(10);
	mls.setDistinctCloud(xyzCloud);
	mls.setNumberOfThreads(3);
	mls.setInputCloud(xyzCloud);
	mls.process(mls_points);

	if(mls_points.size() > 0)
		pcl::io::savePCDFile("pcdDataSet.pcd", mls_points);
}

pcl::PointCloud<pcl::PointNormal> smoothingMLS(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal> mls_points;
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);
	mls.setInputCloud(xyzCloud);
	mls.setPolynomialOrder(2);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	mls.process(mls_points);

	//pcl::io::savePCDFile("bun0-mls.pcd", mls_points);
	return mls_points;
}

pcl::PolygonMesh poisson_recon(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(xyzCloud);
	filter.filter(*filtered);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(8);
	ne.setInputCloud(filtered);
	ne.setRadiusSearch(0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid(*filtered, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);
	cout << "normal estimation complete" << endl;
	cout << "reverse normals' direction" << endl;

	for (size_t i = 0; i < cloud_normals->size(); ++i) {
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}

	cout << "combine points and normals" << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
	concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);
	*/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(xyzCloud);
	normalEstimation.setInputCloud(xyzCloud);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setKSearch(20);
	normalEstimation.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	// Concatenate the obtained point data and normal data
	pcl::concatenateFields(*xyzCloud, *normals, *cloudWithNormals);

	// another kd-tree for reconstruction
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloudWithNormals);

	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(8);
	poisson.setSolverDivide(8);
	poisson.setIsoDivide(8);
	poisson.setPointWeight(4.0f);
	poisson.setInputCloud(cloudWithNormals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	return mesh;
}

pcl::PolygonMesh bsplineFitting(pcl::PointCloud<Point>::Ptr xyzCloud) { //, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
	pcl::on_nurbs::NurbsDataSurface data;
	
	PointCloud2Vector3d(xyzCloud, data.interior);
	pcl::visualization::PointCloudColorHandlerCustom<Point> handler(xyzCloud, 0, 255, 0);
	//viewer->addPointCloud<Point>(xyzCloud, handler, "cloud_cylinder");

	// parameters
	unsigned order(3);
	unsigned refinement(3);
	unsigned iterations(10);
	unsigned mesh_resolution(256);

	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;
	
	// initialize
	printf("surface fitting ...\n");
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(order, &data);
	pcl::on_nurbs::FittingSurface fit(&data, nurbs);
	fit.setQuiet (false); // enable/disable debug output
	// mesh for visualization
	pcl::PolygonMesh mesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh, mesh_resolution);
	//viewer->addPolygonMesh(mesh, mesh_id);

	printf("surface refinement %u\n", refinement);
	for (unsigned i = 0; i < refinement; i++)
	{
		printf("%u ", i);
		fit.refine(0);
		fit.refine(1);
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		//viewer->updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
		//viewer->spinOnce();
	}
	printf("\n");

	printf("surface fitting with final refinement level %u\n", iterations);
	for (unsigned i = 0; i < iterations; i++)
	{
		printf("%u ", i);
		fit.assemble(params);
		fit.solve();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices(fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
		//viewer->updatePolygonMesh<pcl::PointXYZ>(mesh_cloud, mesh_vertices, mesh_id);
		//viewer->spinOnce();
	}
	printf("\n");


	printf("fit B-spline curve\n");
	
	// parameters
	pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	curve_params.addCPsAccuracy = 5e-2;
	curve_params.addCPsIteration = 3;
	curve_params.maxCPs = 200;
	curve_params.accuracy = 1e-3;
	curve_params.iterations = 100;
	
	curve_params.param.closest_point_resolution = 0;
	curve_params.param.closest_point_weight = 1.0;
	curve_params.param.closest_point_sigma2 = 0.1;
	curve_params.param.interior_sigma2 = 0.00001;
	curve_params.param.smooth_concavity = 1.0;
	curve_params.param.smoothness = 1.0;
	
	// initialisation (circular)
	printf("curve fitting ...\n");
	pcl::on_nurbs::NurbsDataCurve2d curve_data;
	curve_data.interior = data.interior_param;
	curve_data.interior_weight_function.push_back(true);
	ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(order, curve_data.interior);
	
	// curve fitting
	pcl::on_nurbs::FittingCurve2dASDM curve_fit(&curve_data, curve_nurbs);
	// curve_fit.setQuiet (false); // enable/disable debug output
	curve_fit.fitting(curve_params);
	//visualizeCurve(curve_fit.m_nurbs, fit.m_nurbs, viewer);

	// triangulation of trimmed surface
	
	printf("triangulate trimmed surface ...\n");
	//viewer->removePolygonMesh(mesh_id);
	pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(fit.m_nurbs, curve_fit.m_nurbs, mesh,
	                                                                   mesh_resolution);
	//viewer->addPolygonMesh(mesh, mesh_id);

	printf("... done.\n");
	//viewer->spin();

	return mesh;
}

void PointCloud2Vector3d(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d & data)
{
	for (unsigned i = 0; i < cloud->size(); i++)
	{
		Point& p = cloud->at(i);
		if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
			data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

void visualizeCurve(ON_NurbsCurve & curve, ON_NurbsSurface & surface, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::on_nurbs::Triangulation::convertCurve2PointCloud(curve, surface, curve_cloud, 4);
	for (std::size_t i = 0; i < curve_cloud->size() - 1; i++)
	{
		pcl::PointXYZRGB & p1 = curve_cloud->at(i);
		pcl::PointXYZRGB & p2 = curve_cloud->at(i + 1);
		std::ostringstream os;
		os << "line" << i;
		viewer->removeShape(os.str());
		viewer->addLine<pcl::PointXYZRGB>(p1, p2, 1.0, 0.0, 0.0, os.str());
	}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr curve_cps(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < curve.CVCount(); i++)
	{
		ON_3dPoint p1;
		curve.GetCV(i, p1);
		
		double pnt[3];
		surface.Evaluate(p1.x, p1.y, 0, 3, pnt);
		pcl::PointXYZRGB p2;
		p2.x = float(pnt[0]);
		p2.y = float(pnt[1]);
		p2.z = float(pnt[2]);
		
		p2.r = 255;
		p2.g = 0;
		p2.b = 0;
		
		curve_cps->push_back(p2);
	}
	viewer->removePointCloud("cloud_cps");
	viewer->addPointCloud(curve_cps, "cloud_cps");
}