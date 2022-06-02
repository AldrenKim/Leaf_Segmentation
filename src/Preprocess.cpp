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
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/console/parse.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

pcl::PolygonMesh edgeSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::MeshSmoothingLaplacianVTK item;
	pcl::PolygonMesh res = triangulationGreedyProjection(xyzCloud);
	pcl::PolygonMeshConstPtr d(new pcl::PolygonMesh(res));
	item.setInputMesh(d);
	item.setBoundarySmoothing(true);
	item.setEdgeAngle(M_PI / 18);
	item.setFeatureAngle(M_PI / 3);
	item.setFeatureEdgeSmoothing(true);
	item.setNumIter(3);
	item.process(res);
	return res;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleFile(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::VoxelGrid<pcl::PCLPointCloud2> dple;
	pcl::PCLPointCloud2::Ptr input(new pcl::PCLPointCloud2());


	printf("Pointcloud before filtering: %u, %u\n", xyzCloud->width, xyzCloud->height);

	pcl::toPCLPointCloud2(*xyzCloud, *input);
	dple.setInputCloud(input);
	dple.setLeafSize(0.3f, 0.3f, 0.3f); //=3%
	dple.filter(*input);

	printf("Pointcloud after filtering: %u, %u\n", input->width, input->height);

	pcl::fromPCLPointCloud2(*input, *xyzCloud);

	if (xyzCloud->size() > 0)
		pcl::io::savePCDFile("pcdDownsample.pcd", *xyzCloud);

	return xyzCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr morph(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud){
	pcl::Morphology<pcl::PointXYZI> *mp = new pcl::Morphology<pcl::PointXYZI>();
	pcl::PointCloud<pcl::PointXYZI>::Ptr xyzRGBCloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr xyzRGBCloudOut(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);

	xyzRGBCloud->points.resize(xyzCloud->size());
	for (size_t i = 0; i < xyzRGBCloud->points.size(); i++){
		xyzRGBCloud->points[i].x = xyzCloud->points[i].x;
		xyzRGBCloud->points[i].y = xyzCloud->points[i].y;
		xyzRGBCloud->points[i].z = xyzCloud->points[i].z;
		xyzRGBCloud->points[i].intensity = 1.0;
	}
	mp->setInputCloud(xyzRGBCloud);
	mp->closingBinary(*xyzRGBCloudOut);

	res->points.resize(xyzRGBCloudOut->size());
	for (size_t i = 0; i < xyzRGBCloudOut->points.size(); i++){
		res->points[i].x = xyzRGBCloudOut->points[i].x;
		res->points[i].y = xyzRGBCloudOut->points[i].y;
		res->points[i].z = xyzRGBCloudOut->points[i].z;
	}

	return res;
}

void edgeSmoothing1(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::PointCloud<pcl::PointNormal>mls_points;
	pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_for_points(new pcl::search::KdTree<pcl::PointXYZ>);


	mls.setComputeNormals(true);
	mls.setSearchMethod(kdtree_for_points);
	mls.setInputCloud(xyzCloud);
	mls.setPolynomialOrder(2);
	mls.setSearchRadius(0.5);
	mls.setSqrGaussParam(0.25);
	mls.setNumberOfThreads(3);


	//mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::VOXEL_GRID_DILATION);
	//mls.setUpsamplingStepSize(0.5);
	// For VOXEL_Grid_Dilation
	//mls.setDilationVoxelSize(0.2);
	//mls.setDilationIterations(5);
	//For Random_Uniform_Density
	//mls.setPointDensity(10);
	//mls.setDistinctCloud(xyzCloud);
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

pcl::PolygonMesh poisson_recon_MLS(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(xyzCloud);
	normalEstimation.setNumberOfThreads(8);
	normalEstimation.setInputCloud(xyzCloud);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setKSearch(10);
	normalEstimation.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
	// Concatenate the obtained point data and normal data
	pcl::concatenateFields(*xyzCloud, *normals, *cloudWithNormals);

	// another kd-tree for reconstruction
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloudWithNormals);

	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(4);
	poisson.setSolverDivide(5);
	poisson.setIsoDivide(5);
	poisson.setPointWeight(8.0f);
	poisson.setInputCloud(cloudWithNormals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	return mesh;
}
pcl::PolygonMesh poisson_recon_MLS_Pass(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PassThrough<pcl::PointXYZ> filter;
	filter.setInputCloud(xyzCloud);
	filter.filter(*filtered);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(8);
	ne.setInputCloud(filtered);
	ne.setRadiusSearch(0.01);

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

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(4);
	poisson.setSolverDivide(5);
	poisson.setIsoDivide(5);
	poisson.setPointWeight(8.0f);
	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	return mesh;
}
pcl::PolygonMesh poisson_recon_MLS_Pass_NE(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
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

	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>());
	result->resize(cloud_smoothed_normals->size());

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(8);
	poisson.setSolverDivide(10);
	poisson.setIsoDivide(10);
	poisson.setPointWeight(10.0f);
	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	return mesh;
}
pcl::PolygonMesh poisson_recon_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
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
	poisson.setSolverDivide(10);
	poisson.setIsoDivide(10);
	poisson.setPointWeight(10.0f);
	poisson.setInputCloud(cloudWithNormals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	return mesh;
}
pcl::PolygonMesh poisson_recon_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	
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
	
	cout << "begin poisson reconstruction" << endl;
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(8);
	poisson.setSolverDivide(10);
	poisson.setIsoDivide(10);
	poisson.setPointWeight(10.0f);
	poisson.setInputCloud(cloud_smoothed_normals);
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
	unsigned mesh_resolution(128);

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
	curve_params.accuracy = 1.5e3; //should not be negative
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

pcl::PointCloud<pcl::PointXYZ>::Ptr ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
		model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(xyzCloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
	std::vector<int> inliers;

	ransac.setDistanceThreshold(0.1);
	ransac.computeModel();
	ransac.getInliers(inliers);

	pcl::copyPointCloud(*xyzCloud, inliers, *final);

	return final;
}

void supervoxels_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud, pcl::visualization::PCLVisualizer::Ptr& viewer) {
	PointCloudT::Ptr cloud(new PointCloudT);
	cloud->points.resize(xyzCloud->size());

	for (size_t i = 0; i < xyzCloud->points.size(); i++) {
		cloud->points[i].x = xyzCloud->points[i].x;
		cloud->points[i].y = xyzCloud->points[i].y;
		cloud->points[i].z = xyzCloud->points[i].z;
	}

	bool disable_transform = false;

	float voxel_resolution = 0.04f;

	float seed_resolution = 0.2f;

	float color_importance = 0.1f;

	float spatial_importance = 0.4f;

	float normal_importance = 0.5f;

	//////////////////////////////  //////////////////////////////
	////// This is how to use supervoxels
	//////////////////////////////  //////////////////////////////

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	if (disable_transform)
		super.setUseSingleCameraTransform(false);
	super.setInputCloud(cloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);

	std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

	pcl::console::print_highlight("Extracting supervoxels!\n");
	super.extract(supervoxel_clusters);
	pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());

	viewer->setBackgroundColor(0, 0, 0);

	PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
	viewer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

	PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
	viewer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

	PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
	//We have this disabled so graph is easy to see, uncomment to see supervoxel normals
	//viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

	pcl::console::print_highlight("Getting supervoxel adjacency\n");
	std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	//To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
	for (auto label_itr = supervoxel_adjacency.cbegin(); label_itr != supervoxel_adjacency.cend(); )
	{
		//First get the label
		std::uint32_t supervoxel_label = label_itr->first;
		//Now get the supervoxel corresponding to the label
		pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

		//Now we need to iterate through the adjacent supervoxels and make a point cloud of them
		PointCloudT adjacent_supervoxel_centers;
		for (auto adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr)
		{
			pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
			adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
		}
		//Now we make a name for this polygon
		std::stringstream ss;
		ss << "supervoxel_" << supervoxel_label;
		//This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
		addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
		//Move iterator forward to next label
		label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
	pcl::console::print_highlight("Supervoxels done\n");
}

void addSupervoxelConnectionsToViewer(
	PointT& supervoxel_center,
	PointCloudT& adjacent_supervoxel_centers,
	std::string supervoxel_name,
	pcl::visualization::PCLVisualizer::Ptr& viewer ) {
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	//Iterate through all adjacent points, and add a center point to adjacent point pair
	for (auto adjacent_itr = adjacent_supervoxel_centers.begin(); adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
	{
		points->InsertNextPoint(supervoxel_center.data);
		points->InsertNextPoint(adjacent_itr->data);
	}
	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// Add the points to the dataset
	polyData->SetPoints(points);
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);
	cells->InsertNextCell(polyLine);
	// Add the lines to the dataset
	polyData->SetLines(cells);
	viewer->addModelFromPolyData(polyData, supervoxel_name);
}