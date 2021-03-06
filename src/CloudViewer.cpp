#include "CloudViewer.h"

CloudViewer::CloudViewer(QWidget *parent)
	: QMainWindow(parent) {
	ui.setupUi(this);

	/***** Slots connection of QMenuBar and QToolBar *****/
	// File (connect)
	QObject::connect(ui.openAction, &QAction::triggered, this, &CloudViewer::open);
	QObject::connect(ui.addAction, &QAction::triggered, this, &CloudViewer::add);
	QObject::connect(ui.clearAction, &QAction::triggered, this, &CloudViewer::clear);

	ui.saveAction->setData(QVariant(false));       // isSaveBinary = false
	ui.saveBinaryAction->setData(QVariant(true));  // isSaveBinary = true
	connect(ui.saveAction, SIGNAL(triggered()), this, SLOT(save()));
	connect(ui.saveBinaryAction, SIGNAL(triggered()), this, SLOT(save()));
	QObject::connect(ui.exitAction, &QAction::triggered, this, &CloudViewer::exit);
	// Display (connect)
	QObject::connect(ui.pointcolorAction, &QAction::triggered, this, &CloudViewer::pointcolorChanged);
	QObject::connect(ui.bgcolorAction, &QAction::triggered, this, &CloudViewer::bgcolorChanged);
	QObject::connect(ui.mainviewAction, &QAction::triggered, this, &CloudViewer::mainview);
	QObject::connect(ui.leftviewAction, &QAction::triggered, this, &CloudViewer::leftview);
	QObject::connect(ui.topviewAction, &QAction::triggered, this, &CloudViewer::topview);
	// Generate (connect)
	QObject::connect(ui.cubeAction, &QAction::triggered, this, &CloudViewer::cube);
	QObject::connect(ui.sphereAction, &QAction::triggered, this, &CloudViewer::createSphere);
	QObject::connect(ui.cylinderAction, &QAction::triggered, this, &CloudViewer::createCylinder);
	// Process (connect)
	QObject::connect(ui.meshsurfaceAction, &QAction::triggered, this, &CloudViewer::convertSurface);
	QObject::connect(ui.wireframeAction, &QAction::triggered, this, &CloudViewer::convertWireframe);
	// Option (connect)
	ui.windowsThemeAction->setData(QVariant(CLOUDVIEWER_THEME_WINDOWS));
	ui.darculaThemeAction->setData(QVariant(CLOUDVIEWER_THEME_DARCULA));
	ui.englishAction->setData(QVariant(CLOUDVIEWER_LANG_ENGLISH));
	ui.chineseAction->setData(QVariant(CLOUDVIEWER_LANG_CHINESE));
	connect(ui.windowsThemeAction, SIGNAL(triggered()), this, SLOT(changeTheme()));
	connect(ui.darculaThemeAction, SIGNAL(triggered()), this, SLOT(changeTheme()));
	connect(ui.englishAction, SIGNAL(triggered()), this, SLOT(changeLanguage()));
	connect(ui.chineseAction, SIGNAL(triggered()), this, SLOT(changeLanguage()));
	// About (connect)
	QObject::connect(ui.aboutAction, &QAction::triggered, this, &CloudViewer::about);
	QObject::connect(ui.helpAction, &QAction::triggered, this, &CloudViewer::help);

	//Pre-process (connect)
	QObject::connect(ui.edgeDetectAction, &QAction::triggered, this, &CloudViewer::edgeDetect);
	QObject::connect(ui.edgeSmoothAction, &QAction::triggered, this, &CloudViewer::edgeSmooth);
	QObject::connect(ui.downsampleAction, &QAction::triggered, this, &CloudViewer::downsample);

	//Surface Reconstruction (connect)

	QObject::connect(ui.poissonAction, &QAction::triggered, this, &CloudViewer::poisson);
	QObject::connect(ui.bsplineAction, &QAction::triggered, this, &CloudViewer::bspline);
	QObject::connect(ui.pass_through_MLS_PoissonAction, &QAction::triggered, this, &CloudViewer::poisson2v);
	QObject::connect(ui.pass_through_MLS_NormalsAction, &QAction::triggered, this, &CloudViewer::poisson3v);
	QObject::connect(ui.mlsAction, &QAction::triggered, this, &CloudViewer::mls);

	//Surface Reconstruction (connect)

	QObject::connect(ui.actionRANSAC_Plane, &QAction::triggered, this, &CloudViewer::ransac_plane);
	QObject::connect(ui.actionSupervoxels, &QAction::triggered, this, &CloudViewer::supervoxels);

	/***** Slots connection of RGB widget *****/
	// Random color (connect)
	connect(ui.colorBtn, SIGNAL(clicked()), this, SLOT(colorBtnPressed()));
	// Connection between RGB slider and RGB value (connect)
	connect(ui.rSlider, SIGNAL(valueChanged(int)), this, SLOT(rSliderChanged(int)));
	connect(ui.gSlider, SIGNAL(valueChanged(int)), this, SLOT(gSliderChanged(int)));
	connect(ui.bSlider, SIGNAL(valueChanged(int)), this, SLOT(bSliderChanged(int)));
	// RGB slider released (connect)
	connect(ui.rSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(ui.gSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(ui.bSlider, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	// Change size of cloud (connect)
	connect(ui.pSlider, SIGNAL(valueChanged(int)), this, SLOT(pSliderChanged(int)));
	connect(ui.pSlider, SIGNAL(sliderReleased()), this, SLOT(psliderReleased()));
	// Checkbox for coordinate and background color (connect)
	connect(ui.bgcCbx, SIGNAL(stateChanged(int)), this, SLOT(bgcCbxChecked(int)));

	/***** Slots connection of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked (connect)
	connect(ui.dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelected(QTreeWidgetItem*, int)));
	// Item in dataTree is right-clicked
	connect(ui.dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenu(const QPoint&)));


	// Initialization
	initial();
}

CloudViewer::~CloudViewer() {

}
//Pre-process
int CloudViewer::edgeDetect() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	edgeSmoothing1(xyzCloud);
	timeCostSecond = timeOff();
	long size = xyzCloud->size();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << size << "\n";
	return 0;
}
int CloudViewer::edgeSmooth(){
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PolygonMesh mesh = edgeSmoothing(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-greedy-projection");
	viewer->setRepresentationToSurfaceForAllActors();

	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";
	return 0;
}
int CloudViewer::downsample(){
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}
	
	xyzCloud = downsampleFile(xyzCloud);
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";
	return 0;
}

//Surface Resconstruction
int CloudViewer::poisson() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PolygonMesh mesh = poisson_recon_MLS(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-poisson");
	viewer->setRepresentationToSurfaceForAllActors();

	/* //Add MLS Cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	*cloud_with_normals = smoothingMLS(xyzCloud);

	viewer->addPointCloud<pcl::PointNormal>(cloud_with_normals, "mls-cloud");
	viewer->updatePointCloud<pcl::PointNormal>(cloud_with_normals, "mls-cloud");

	// update tree widget
	QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList()
		<< toQString("MLS Cloud"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);


	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();
	*/

	//pcl::PolygonMesh mesh = bsplineFitting(xyzCloud);
	//viewer->addPolygonMesh(mesh, "mesh_nurbs");
	//viewer->setRepresentationToSurfaceForAllActors();


	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";
	return 0;
}
int CloudViewer::bspline() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PolygonMesh mesh = bsplineFitting(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-bsplineFitting");
	viewer->setRepresentationToSurfaceForAllActors();

	/* //Add MLS Cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	*cloud_with_normals = smoothingMLS(xyzCloud);

	viewer->addPointCloud<pcl::PointNormal>(cloud_with_normals, "mls-cloud");
	viewer->updatePointCloud<pcl::PointNormal>(cloud_with_normals, "mls-cloud");

	// update tree widget
	QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList()
		<< toQString("MLS Cloud"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);


	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();
	*/

	//pcl::PolygonMesh mesh = bsplineFitting(xyzCloud);
	//viewer->addPolygonMesh(mesh, "mesh_nurbs");
	//viewer->setRepresentationToSurfaceForAllActors();


	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";
	return 0;
}
int CloudViewer::poisson3v() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PolygonMesh mesh = poisson_recon_MLS_Pass_NE(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-poisson-mls-pass-NE");
	viewer->setRepresentationToSurfaceForAllActors();

	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";
	return 0;
}
int CloudViewer::poisson2v() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PolygonMesh mesh = poisson_recon_MLS_Pass(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-poisson-mls-pass");
	viewer->setRepresentationToSurfaceForAllActors();

	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	};
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";
	return 0;
}
int CloudViewer::mls() {
	//insert code here
	return 0;
}

void CloudViewer::doOpen(const QStringList& filePathList) {
	// Open point cloud file one by one
	for (int i = 0; i != filePathList.size(); i++) {
		timeStart(); // time start
		mycloud.cloud.reset(new PointCloudT); // Reset cloud
		QFileInfo fileInfo(filePathList[i]);
		std::string filePath = fromQString(fileInfo.filePath());
		std::string fileName = fromQString(fileInfo.fileName());
		
		// begin loading
		ui.statusBar->showMessage(
			fileInfo.fileName() + ": " + QString::number(i) + "/" + QString::number(filePathList.size())
			+ " point cloud loading..."
		);

		mycloud = fileIO.load(fileInfo);
		if (!mycloud.isValid) {
			// TODO: deal with the error, print error info in console?
			debug("invalid cloud.");
			continue;
		}
		mycloud.viewer = viewer;
		mycloud_vec.push_back(mycloud);

		timeCostSecond = timeOff(); // time off

		// update tree widget
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList()
			<< toQString(mycloud.fileName));
		cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		ui.dataTree->addTopLevelItem(cloudName);

		total_points += mycloud.cloud->points.size();
		cout << "Number of points before: " << mycloud.cloud->points.size() << "\n";
	}
	ui.statusBar->showMessage("");
	showPointcloudAdd();
}

// Open point cloud
void CloudViewer::open() {
	QStringList filePathList = QFileDialog::getOpenFileNames(
		this,
		tr("Open point cloud file"),
		toQString(mycloud.fileDir),
		toQString(fileIO.getInputFormatsStr())
	);
	if (filePathList.isEmpty()) return;

	// Clear cache
	// TODO: abstract a function
	mycloud_vec.clear();
	total_points = 0;
	ui.dataTree->clear();
	viewer->removeAllPointClouds();

	doOpen(filePathList);
}

// Add Point Cloud
void CloudViewer::add() {
	QStringList filePathList = QFileDialog::getOpenFileNames(
		this,
		tr("Add point cloud file"),
		toQString(mycloud.fileDir),
		toQString(fileIO.getInputFormatsStr())
	);
	if (filePathList.isEmpty()) return;

	doOpen(filePathList);
}

// Clear all point clouds
void CloudViewer::clear() {
	mycloud_vec.clear();  //????????????????????????????????????
	viewer->removeAllPointClouds();  //???viewer?????????????????????
	viewer->removeAllShapes(); //??????remove?????????
	ui.dataTree->clear();  //???dataTree??????

	QStringList header;
	header << "Property" << "Value";

	//????????????

	setWindowTitle("3D Leaf Viewer");  //??????????????????
	showPointcloud();  //????????????
}

// Save point cloud
void CloudViewer::save() {
	if (!mycloud.isValid) {
		QMessageBox::critical(this, tr("Saving file error"),
			tr("There is no point cloud to save"));
		return;
	}

	// get binary flag from sender()
	QAction *action = qobject_cast<QAction *>(sender());
	QVariant v = action->data();
	bool isSaveBinary = (bool)v.value<bool>();

	QString selectedFilter = toQString(fileIO.outputFiltersMap.at(mycloud.fileSuffix));
	QString saveFilePath = QFileDialog::getSaveFileName(
		this,                                    // parent
		toQString("Save point cloud" + string(isSaveBinary ? " (binary)": "")), // caption
		toQString(mycloud.filePath),             // dir
		toQString(fileIO.getOutputFormatsStr()), // filter
		&selectedFilter                          // selected filter
	);
	if (saveFilePath.isEmpty()) return;

	QFileInfo fileInfo(saveFilePath);
	QString saveFileName = fileInfo.fileName();
	string saveFilePathStd = fromQString(saveFilePath);
	string saveFileNameStd = fromQString(saveFileName);

	if (mycloud_vec.size() > 1) {
		savemulti(fileInfo, isSaveBinary);
		return;
	}

	bool saveStatus = fileIO.save(mycloud, fileInfo, isSaveBinary);
	if (!saveStatus) {
		QMessageBox::critical(this, tr("Saving file error"),
			tr("We can not save the file"));
		return;
	}


	setWindowTitle(saveFilePath + " - 3D Leaf Viewer");
	QMessageBox::information(this, tr("save point cloud file"),
		toQString("Save " + saveFileNameStd + " successfully!"));
}

// Save multi point cloud
void CloudViewer::savemulti(const QFileInfo& fileInfo, bool isSaveBinary) {
	string subname = fromQString(fileInfo.fileName());
	QString saveFilePath = fileInfo.filePath();
	PointCloudT::Ptr multi_cloud;
	multi_cloud.reset(new PointCloudT);
	multi_cloud->height = 1;
	int sum = 0;
	for (auto c : mycloud_vec) {
		sum += c.cloud->points.size();
	}
	multi_cloud->width = sum;
	multi_cloud->resize(multi_cloud->height * multi_cloud->width);
	int k = 0;
	for (int i = 0; i != mycloud_vec.size(); ++i) {
		// ??????cloudvec[i]->points.size()???cloudvec[i]->size()?????????
		for (int j = 0; j != mycloud_vec[i].cloud->points.size(); ++j) {
			multi_cloud->points[k].x = mycloud_vec[i].cloud->points[j].x;
			multi_cloud->points[k].y = mycloud_vec[i].cloud->points[j].y;
			multi_cloud->points[k].z = mycloud_vec[i].cloud->points[j].z;
			multi_cloud->points[k].r = mycloud_vec[i].cloud->points[j].r;
			multi_cloud->points[k].g = mycloud_vec[i].cloud->points[j].g;
			multi_cloud->points[k].b = mycloud_vec[i].cloud->points[j].b;
			k++;
		}
	}
	
	MyCloud multiMyCloud;
	multiMyCloud.cloud = multi_cloud;
	multiMyCloud.isValid = true;

	// save multi_cloud
	bool saveStatus = fileIO.save(multiMyCloud, fileInfo, isSaveBinary);
	if (!saveStatus) {
		QMessageBox::critical(this, tr("Saving file error"),
			tr("We can not save the file"));
		return;
	}

	// ??????????????? multi_cloud ??????????????? mycloud,????????????????????????????????????
	mycloud.cloud = multi_cloud;
	mycloud.filePath = fromQString(saveFilePath);
	mycloud.fileName = subname;

	setWindowTitle(saveFilePath + " - 3D Leaf Viewer");
	QMessageBox::information(this, tr("save point cloud file"), toQString("Save " + subname + " successfully!"));
}

//????????????
void CloudViewer::exit() {
	this->close();
}

// Generate cube
void CloudViewer::cube() {
	mycloud.cloud.reset(new PointCloudT);
	total_points = 0;
	ui.dataTree->clear();  //????????????????????????item
	viewer->removeAllPointClouds();  //???viewer?????????????????????
	mycloud_vec.clear();  //??????????????????

	mycloud.cloud->width = 50000;         // ???????????????
	mycloud.cloud->height = 1;            // ????????????????????????1???????????????????????????
	mycloud.cloud->is_dense = false;
	mycloud.cloud->resize(mycloud.cloud->width * mycloud.cloud->height);     // ??????????????????
	for (size_t i = 0; i != mycloud.cloud->size(); ++i)
	{
		mycloud.cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		mycloud.cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		mycloud.cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		mycloud.cloud->points[i].r = red;
		mycloud.cloud->points[i].g = green;
		mycloud.cloud->points[i].b = blue;
	}
	//?????????????????????
	QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("cube"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);


	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();
}

//?????????
void CloudViewer::initial() {
	//???????????????
	setWindowIcon(QIcon(tr(":/Resources/images/icon.png")));
	setWindowTitle(tr("3D Leaf Viewer"));

	//???????????????
	mycloud.cloud.reset(new PointCloudT);
	mycloud.cloud->resize(1);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//viewer->addPointCloud(cloud, "cloud");

	ui.screen->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());
	ui.screen->update();

	ui.dataTree->setSelectionMode(QAbstractItemView::ExtendedSelection); // ?????? dataTree ????????????

	// ??????????????????
	QString qss = custom_qss;
	qApp->setStyleSheet(qss);


	// ????????????????????? dark
	viewer->setBackgroundColor(45 / 255.0, 51 / 255.0, 25 / 255.0);

}

//????????????????????????????????????
void CloudViewer::showPointcloud() {
	for (int i = 0; i != mycloud_vec.size(); i++)
	{
		viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
	}
	//viewer->resetCamera();
	ui.screen->update();
}

//???????????????viewer,???????????????
void CloudViewer::showPointcloudAdd() {
	for (int i = 0; i != mycloud_vec.size(); i++) {
		viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
		viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
	}
	viewer->resetCamera();
	ui.screen->update();
}

void CloudViewer::setCloudColor(unsigned int r, unsigned int g, unsigned int b) {
	// Set the new color
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		mycloud.cloud->points[i].r = r;
		mycloud.cloud->points[i].g = g;
		mycloud.cloud->points[i].b = b;
		mycloud.cloud->points[i].a = 255;
	}
}

//??????
void CloudViewer::about() {
	AboutWin *aboutwin = new AboutWin(this);
	aboutwin->setModal(true);
	aboutwin->show();
}

//??????
void CloudViewer::help() {
	QDesktopServices::openUrl(QUrl(QLatin1String("http://nightn.com/cloudviewer")));
}

//??????????????????
void CloudViewer::createSphere() {
	bool flag;
	mycloud.cloud.reset(new PointCloudT);
	ui.dataTree->clear();  //????????????????????????item
	viewer->removeAllShapes();
	mycloud_vec.clear();  //??????????????????

	pcl::ModelCoefficients sphere_coeff;
	sphere_coeff.values.resize(4);
	sphere_coeff.values[0] = 0;
	sphere_coeff.values[1] = 0;
	sphere_coeff.values[2] = 0;
	sphere_coeff.values[3] = 7;
	
	flag = viewer->addSphere(sphere_coeff, "sphere1");

	viewer->resetCamera();
	ui.screen->update();


	QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("sphere"));
	cloudName->setIcon(0, QIcon(":/Resources/images/sphere.png"));
	ui.dataTree->addTopLevelItem(cloudName);


	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();

}

void CloudViewer::createCylinder() {
	bool flag;
	mycloud.cloud.reset(new PointCloudT);
	ui.dataTree->clear();  //????????????????????????item
	viewer->removeAllShapes();
	mycloud_vec.clear();  //??????????????????

	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize(7);
	cylinder_coeff.values[0] = 0;
	cylinder_coeff.values[1] = 0;
	cylinder_coeff.values[2] = 0;
	cylinder_coeff.values[3] = 2.6;
	cylinder_coeff.values[4] = 4.6;
	cylinder_coeff.values[5] = 7.6;
	cylinder_coeff.values[6] = 11.5f;

	flag = viewer->addCylinder(cylinder_coeff);

	viewer->resetCamera();
	ui.screen->update();

	QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("Cylinder"));
	cloudName->setIcon(0, QIcon(":/Resources/images/cylinder.png"));
	ui.dataTree->addTopLevelItem(cloudName);


	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();


}

// Change theme: Windows/Darcula
void CloudViewer::changeTheme() {
	QAction *action = qobject_cast<QAction *>(sender());
	QVariant v = action->data();
	int theme = (int)v.value<int>();

	QColor colorLight(241, 241, 241, 255);
	QColor colorDark(0, 0, 0, 255);
	QString qss;

	switch (theme) {
		case CLOUDVIEWER_THEME_WINDOWS: {
			qss = windows_qss;
			for (int i = 0; i != mycloud_vec.size(); i++){
				if (ui.dataTree->topLevelItem(i)->textColor(0) == colorLight){
					ui.dataTree->topLevelItem(i)->setTextColor(0, colorDark);
				}
			}
			theme_id = 0;
			break;
		}
		case CLOUDVIEWER_THEME_DARCULA: {
			qss = darcula_qss;
			for (int i = 0; i != mycloud_vec.size(); i++){
				if (ui.dataTree->topLevelItem(i)->textColor(0) == colorDark){
					ui.dataTree->topLevelItem(i)->setTextColor(0, colorLight);
				}
			}
			theme_id = 1;
			break;
		}
	}
	qApp->setStyleSheet(qss);
}

// Change language: English/Chinese
void CloudViewer::changeLanguage() {
	QAction *action = qobject_cast<QAction *>(sender());
	QVariant v = action->data();
	int language = (int)v.value<int>();

	switch (language) {
		case CLOUDVIEWER_LANG_ENGLISH: {
			break;
		}
		case CLOUDVIEWER_LANG_CHINESE: {
			break;
		}
	}
}


/*********************************************/
/*****************???????????????*****************/
/********************************************/
void CloudViewer::colorBtnPressed() {
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	// ??????????????????????????????????????????????????????????????????????????????
	if (selected_item_count == 0){
		for (int i = 0; i != mycloud_vec.size(); i++){
			for (int j = 0; j != mycloud_vec[i].cloud->points.size(); j++){
				mycloud_vec[i].cloud->points[j].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				mycloud_vec[i].cloud->points[j].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				mycloud_vec[i].cloud->points[j].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			}
		}

		// ????????????

	}
	else{
		for (int i = 0; i != selected_item_count; i++){
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			for (int j = 0; j != mycloud_vec[cloud_id].cloud->size(); j++){
				mycloud_vec[cloud_id].cloud->points[j].r = red;
				mycloud_vec[cloud_id].cloud->points[j].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
				mycloud_vec[cloud_id].cloud->points[j].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
			}
		}

		// ????????????
	}
	showPointcloud();
}

void CloudViewer::RGBsliderReleased() {
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	// ??????????????????????????????????????????????????????????????????????????????
	if (selected_item_count == 0) {
		for (int i = 0; i != mycloud_vec.size(); i++) {
			mycloud_vec[i].setPointColor(red, green, blue);
		}

		// ????????????
	}
	else{
		for (int i = 0; i != selected_item_count; i++) {
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			mycloud_vec[cloud_id].setPointColor(red, green, blue);
		}
		// ????????????
	}
	showPointcloud();
}

//???????????????????????????
void CloudViewer::psliderReleased() {
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0){
		for (int i = 0; i != mycloud_vec.size(); i++){
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				p, mycloud_vec[i].cloudId);
		}
		// ????????????
	}
	else{
		for (int i = 0; i != selected_item_count; i++){
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
				p, mycloud_vec[i].cloudId);
		}
		// ????????????
	}
	ui.screen->update();
}

void CloudViewer::pSliderChanged(int value) {
	p = value;
	ui.sizeLCD->display(value);

}

void CloudViewer::rSliderChanged(int value) {
	red = value;
	ui.rLCD->display(value);
}

void CloudViewer::gSliderChanged(int value) {
	green = value;
	ui.gLCD->display(value);
}

void CloudViewer::bSliderChanged(int value) {
	blue = value;
	ui.bLCD->display(value);
}

void CloudViewer::cooCbxChecked(int value) {
	switch (value) {
		case 0: {
			viewer->removeCoordinateSystem();
			break;
		}
		case 2: {
			viewer->addCoordinateSystem();
			break;
		}
	}
	ui.screen->update();
}

void CloudViewer::bgcCbxChecked(int value) {
	switch (value) {
		case 0: {
			viewer->setBackgroundColor(45 / 255.0, 51 / 255.0, 25 / 255.0);
			break;
		}
		case 2: {
			//????????????setBackgroundColor()????????????0-1???double?????????
			viewer->setBackgroundColor(214 / 255.0, 229 / 255.0, 227 / 255.0);
			break;
		}
	}
	ui.screen->update();
}

// ???????????????????????????????????????
void CloudViewer::pointcolorChanged() {
	QColor color = QColorDialog::getColor(Qt::white, this, "Select color for point cloud");

	if (color.isValid()) {
		//QAction* action = dynamic_cast<QAction*>(sender());
		//if (action != ui.pointcolorAction) //?????????????????????????????? dataTree
		QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
		int selected_item_count = ui.dataTree->selectedItems().size();
		if (selected_item_count == 0) {
			for (int i = 0; i != mycloud_vec.size(); ++i) {
				mycloud_vec[i].setPointColor(color.red(), color.green(), color.blue());
			}
			// ????????????
		}
		else {
			for (int i = 0; i != selected_item_count; i++) {
				int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
				mycloud_vec[cloud_id].setPointColor(color.red(), color.green(), color.blue());
			}
			// ????????????
		}
		//????????????????????????RGB????????????
		ui.rSlider->setValue(color.red());
		ui.gSlider->setValue(color.green());
		ui.bSlider->setValue(color.blue());

		showPointcloud();
	}
}

//???????????????????????????????????????
void CloudViewer::bgcolorChanged() {
	QColor color = QColorDialog::getColor(Qt::white, this,
		"Select color for point cloud");
	if (color.isValid()) {
		viewer->setBackgroundColor(color.red() / 255.0,
			color.green() / 255.0, color.blue() / 255.0);
		// ????????????
		showPointcloud();
	}
}

//?????????
void CloudViewer::mainview() {
	viewer->setCameraPosition(0, -1, 0, 0.5, 0.5, 0.5, 0, 0, 1);
	ui.screen->update();
}

void CloudViewer::leftview() {
	viewer->setCameraPosition(-1, 0, 0, 0, 0, 0, 0, 0, 1);
	ui.screen->update();
}

void CloudViewer::topview() {
	viewer->setCameraPosition(0, 0, 1, 0, 0, 0, 0, 1, 0);
	ui.screen->update();
}

//QTreeWidget???item?????????????????????
void CloudViewer::itemSelected(QTreeWidgetItem* item, int count) {
	count = ui.dataTree->indexOfTopLevelItem(item);  //??????item?????????

	for (int i = 0; i != mycloud_vec.size(); i++)
	{
		viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, mycloud_vec[i].cloudId);
	}

	//?????????????????????RGB,?????????????????????
	int cloud_size = mycloud_vec[count].cloud->points.size();
	unsigned int cloud_r = mycloud_vec[count].cloud->points[0].r;
	unsigned int cloud_g = mycloud_vec[count].cloud->points[0].g;
	unsigned int cloud_b = mycloud_vec[count].cloud->points[0].b;
	bool multi_color = true;
	if (mycloud_vec[count].cloud->points.begin()->r == (mycloud_vec[count].cloud->points.end() - 1)->r) //??????????????????????????????????????????????????????
		multi_color = false;

	//??????item??????????????????????????????
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++){
		int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			2, mycloud_vec[i].cloudId);
	}
	//mycloud = mycloud_vec[count];
	ui.screen->update();
}

//QTreeWidget???item?????????????????????
void CloudViewer::popMenu(const QPoint&) {
	QTreeWidgetItem* curItem = ui.dataTree->currentItem(); //??????????????????????????????
	if (curItem == NULL)return;           //????????????????????????????????????treeItem???????????????????????????????????????
	QString name = curItem->text(0);
	int id = ui.dataTree->indexOfTopLevelItem(curItem);
	MyCloud& myCloud = mycloud_vec[id];

	QAction hideItemAction("Hide", this);
	QAction showItemAction("Show", this);
	QAction deleteItemAction("Delete", this);
	QAction changeColorAction("Change color", this);
	// show mode
	QAction pointModeAction("Set point mode", this);
	QAction meshModeAction("Set mesh mode", this);
	QAction pointMeshModeAction("Set point+mesh mode", this);
	pointModeAction.setData(QVariant(CLOUDVIEWER_MODE_POINT));
	meshModeAction.setData(QVariant(CLOUDVIEWER_MODE_MESH));
	pointMeshModeAction.setData(QVariant(CLOUDVIEWER_MODE_POINT_MESH));

	pointModeAction.setCheckable(true);
	meshModeAction.setCheckable(true);
	pointMeshModeAction.setCheckable(true);

	if (myCloud.curMode == "point") {
		pointModeAction.setChecked(true);
	}
	else if (myCloud.curMode == "mesh") {
		meshModeAction.setChecked(true);
	}
	else if (myCloud.curMode == "point+mesh") {
		pointMeshModeAction.setChecked(true);
	}
	
	connect(&hideItemAction, &QAction::triggered, this, &CloudViewer::hideItem);
	connect(&showItemAction, &QAction::triggered, this, &CloudViewer::showItem);
	connect(&deleteItemAction, &QAction::triggered, this, &CloudViewer::deleteItem);
	connect(&changeColorAction, &QAction::triggered, this, &CloudViewer::pointcolorChanged);

	connect(&pointModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));
	connect(&meshModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));
	connect(&pointMeshModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));

	QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&hideItemAction);
	menu.addAction(&showItemAction);
	menu.addAction(&deleteItemAction);
	menu.addAction(&changeColorAction);
	
	menu.addAction(&pointModeAction);
	menu.addAction(&meshModeAction);
	menu.addAction(&pointMeshModeAction);

	if (mycloud_vec[id].visible == true){
		menu.actions()[1]->setVisible(false);
		menu.actions()[0]->setVisible(true);
	}
	else{
		menu.actions()[1]->setVisible(true);
		menu.actions()[0]->setVisible(false);
	}

	const vector<string> modes = myCloud.supportedModes;
	if (std::find(modes.begin(), modes.end(), "point") == modes.end()) {
		menu.actions()[4]->setVisible(false);
	}
	if (std::find(modes.begin(), modes.end(), "mesh") == modes.end()) {
		menu.actions()[5]->setVisible(false);
	}
	if (std::find(modes.begin(), modes.end(), "point+mesh") == modes.end()) {
		menu.actions()[6]->setVisible(false);
	}

	menu.exec(QCursor::pos()); //???????????????????????????
}
void CloudViewer::hideItem() {
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	for (int i = 0; i != ui.dataTree->selectedItems().size(); i++){
		//TODO hide?????????item???????????????????????????item?????????hideItem??? ???????????? ???showItem???
		//QTreeWidgetItem* curItem = ui.dataTree->currentItem();
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui.dataTree->indexOfTopLevelItem(curItem);
		mycloud_vec[id].hide();
		//QMessageBox::information(this, "cloud_id", QString::fromLocal8Bit(cloud_id.c_str()));

		QColor item_color = QColor(112, 122, 132, 255);
		curItem->setTextColor(0, item_color);
		mycloud_vec[id].visible = false;
	}

	ui.screen->update(); //?????????????????????????????????
}

void CloudViewer::showItem() {
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	for (int i = 0; i != ui.dataTree->selectedItems().size(); i++){
		//QTreeWidgetItem* curItem = ui.dataTree->currentItem();
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui.dataTree->indexOfTopLevelItem(curItem);
		// ???cloud_id?????????????????????????????????
		mycloud_vec[id].show();
		QColor item_color;
		if (theme_id == 0){
			item_color = QColor(0, 0, 0, 255);
		}
		else{
			item_color = QColor(241, 241, 241, 255);
		}
		curItem->setTextColor(0, item_color);
		mycloud_vec[id].visible = true;
	}

	ui.screen->update(); //?????????????????????????????????

}

void CloudViewer::deleteItem() {
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	// ui.dataTree->selectedItems().size() ??????????????????????????????????????????????????????????????????????????? selected_item_count
	int selected_item_count = ui.dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++){
		//QTreeWidgetItem* curItem = ui.dataTree->currentItem();
		//QMessageBox::information(this, "itemList's size", QString::number(ui.dataTree->selectedItems().size()));
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui.dataTree->indexOfTopLevelItem(curItem);
		//QMessageBox::information(this, "information", "curItem: " + name + " " + QString::number(id));
		auto it = mycloud_vec.begin() + ui.dataTree->indexOfTopLevelItem(curItem);
		// ?????????????????????????????????????????????
		int delete_points = (*it).cloud->points.size();
		it = mycloud_vec.erase(it);
		//QMessageBox::information(this, "information", QString::number(delete_points) + " " + QString::number(mycloud_vec.size()));

		total_points -= delete_points;

		ui.dataTree->takeTopLevelItem(ui.dataTree->indexOfTopLevelItem(curItem));
	}

	// ?????????????????????????????? id ??????????????????????????????????????????
	viewer->removeAllPointClouds();
	for (int i = 0; i != mycloud_vec.size(); i++)
	{
		viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
		viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
	}

	ui.screen->update();

}

void CloudViewer::setRenderingMode() {
	QAction *action = qobject_cast<QAction *>(sender());
	QVariant v = action->data();
	int mode = (int)v.value<int>();
	string modeStr;

	switch (mode) {
		case CLOUDVIEWER_MODE_POINT: {
			modeStr = "point";
			break;
		}
		case CLOUDVIEWER_MODE_MESH: {
			modeStr = "mesh";
			break;
		}
		case CLOUDVIEWER_MODE_POINT_MESH: {
			modeStr = "point+mesh";
			break;
		}
	}

	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	for (int i = 0; i != ui.dataTree->selectedItems().size(); i++) {
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui.dataTree->indexOfTopLevelItem(curItem);
		MyCloud& myCloud = mycloud_vec[id];
		myCloud.setShowMode(modeStr);
	}
	ui.screen->update();
}

int CloudViewer::convertSurface() {
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}
	
	pcl::PolygonMesh mesh = triangulationGreedyProjection(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-greedy-projection");
	viewer->setRepresentationToSurfaceForAllActors();


	//pcl::PolygonMesh mesh = poisson_recon(xyzCloud);
	//viewer->addPolygonMesh(mesh, "mesh-poisson");
	//viewer->setRepresentationToSurfaceForAllActors();

	//pcl::PolygonMesh mesh = bsplineFitting(xyzCloud);
	//viewer->addPolygonMesh(mesh, "mesh-bsplineFitting");
	//viewer->setRepresentationToSurfaceForAllActors();



	/*pcl::PolygonMesh mesh = bsplineFitting(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-bsplineFitting");
	viewer->setRepresentationToSurfaceForAllActors();*/

	/* //Add MLS Cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	*cloud_with_normals = smoothingMLS(xyzCloud);

	viewer->addPointCloud<pcl::PointNormal>(cloud_with_normals, "mls-cloud");
	viewer->updatePointCloud<pcl::PointNormal>(cloud_with_normals, "mls-cloud");

	// update tree widget
	QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList()
		<< toQString("MLS Cloud"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);


	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();
	*/

	//pcl::PolygonMesh mesh = bsplineFitting(xyzCloud);
	//viewer->addPolygonMesh(mesh, "mesh_nurbs");
	//viewer->setRepresentationToSurfaceForAllActors();

	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	//supervoxels_segmentation(xyzCloud, viewer);

	return 0;
}

int CloudViewer::convertWireframe() {
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PolygonMesh mesh = triangulationGreedyProjection(xyzCloud);
	viewer->addPolygonMesh(mesh, "mesh-greedy-projection");
	viewer->setRepresentationToWireframeForAllActors();

	viewer->removeAllShapes();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
	}
	return 0;
}

void CloudViewer::debug(const string& s) {
	QMessageBox::information(this, tr("Debug"), QString::fromLocal8Bit(s.c_str()));
}

//Segmentation
int CloudViewer::ransac_plane() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr ransac = ransacPlane(xyzCloud);

	mycloud.cloud.reset(new PointCloudT);
	total_points = 0;
	ui.dataTree->clear();  //????????????????????????item
	viewer->removeAllPointClouds();  //???viewer?????????????????????
	mycloud_vec.clear();  //??????????????????

	pcl::copyPointCloud(*ransac, *mycloud.cloud);

	QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("Ransac"));
	cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
	ui.dataTree->addTopLevelItem(cloudName);

	mycloud_vec.push_back(mycloud);
	showPointcloudAdd();

	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";

	return 0;
}

int CloudViewer::supervoxels() {
	timeStart();
	pcl::PointXYZ point;
	xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < mycloud.cloud->size(); i++) {
		point.x = mycloud.cloud->points[i].x;
		point.y = mycloud.cloud->points[i].y;
		point.z = mycloud.cloud->points[i].z;
		xyzCloud->push_back(point);
	}
	if (!xyzCloud) {
		return -1;
	}

	supervoxels_segmentation(xyzCloud, viewer);
	timeCostSecond = timeOff();
	cout << "Time cost: " << timeCostSecond.toStdString() << "\n";
	cout << "Number of points: " << xyzCloud->size() << "\n";

	return 0;
}