//pcl.cpp
#include <pcl.h>
#include <iostream>
#include <windows.h>
#include <numeric>
#include <cmath>
#include <string>
#include <fstream>

#define PL(a) std::cout<<(a)<<std::endl;

PCL::PCL(int option_pcd_type = PCL_XYZ) :

	//----basic point clouds init---
	//cloud_xyz_(new pcl::PointCloud<pcl::PointXYZ>),
	//cloud_xyzrgb_(new pcl::PointCloud<pcl::PointXYZRGB>),
	cloud_xyzrgba_(new pcl::PointCloud<pcl::PointXYZRGBA>),

	//---track()---
	tracker(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(8)),
	coherence(new ApproxNearestPairPointCloudCoherence<RefPointType>),
	distance_coherence(new DistanceCoherence<RefPointType>),
	search(new pcl::search::Octree<RefPointType>(0.01)),
	transed_ref(new Cloud),
	transed_ref_downsampled(new Cloud),
	hsv_coherence(new HSVColorCoherence<RefPointType>),
	cloud_target_remover_ref_(new Cloud),
	cloud_target_remover_ref_downsampled_(new Cloud),
	Rcenter_ref_(new Cloud)
	//,viewer(new pcl::visualization::CloudViewer("PCL"))
	, contact_detector_(new ContactDetector("src/pointpkg/pcd/MS_xyzrgba_common/tool_target_half.pcd"
			,"src/pointpkg/pcd/MS_xyzrgba_common/tool_target_remover.pcd"
			,"src/pointpkg/pcd/MS_xyzrgba_common/tool_1point_tip.pcd"
			))
{
	std::cout << "pcl: constructing" << std::endl;

	pcd_type_ = option_pcd_type;
	PCL::_load_parameters("src/pointpkg/pcl_interface_parameters.txt");


	std::cout << "pcl: constructed" << std::endl;

}

PCL::~PCL() {
	PCL::_save_parameters("src/pointpkg/pcl_interface_parameters.txt");
}

void PCL::init(CoppeliaSimInterface* coppeliasim_interface) {
	coppeliasim_interface_ = coppeliasim_interface;
}


void PCL::update(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	cloud_xyzrgba_->clear();


	//vevtor -> pcd 変換
	PCL::_convert_array_to_pcd(number_of_points, points, color);

	//保存済みデータ読み込み
	//PCL::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba5/1.pcd", number_of_points, color);
	//PCL::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba5/", number_of_points, color);
	//PCL::_read_pcd_file("src/pointpkg/pcd/MS_xyzrgba_common/tool_blue.pcd", number_of_points, color);

	//カメラ->coppeliasimモデル座標変換
	PCL::_transform();
		
	//範囲外除去
	PCL::_filterPassThrough();




	//PCL::_remove_plane();  //xyz only
	//PCL::_compress();
	//PCL::_detect_change(color);  //xyz only
	//PCL::_edit();

	
	//PCL::_recognite(); // to be improved

	//接触判定　粒子フィルタ
	//PCL::_detect_contact_with_particlefilter();


	//接触判定　CoppeliaSim
	PCL::_detect_contact_with_coppeliasim();




	//新規window上に点群描画．PCL::PCL()のviewerの初期化必要
	//PCL::drawResult();

	//pcdデータ（cloud_xyzrgba_）保存．  保存場所指定は最後に"/"忘れずに
	//PCL::_write_pcd_file("src/pointpkg/pcd/MS_xyzrgba11/");

	//pcd->vevtor 変換
	PCL::_convert_pcd_to_array(number_of_points, points, color);

	counter++;
	//std::cout << counter << std::endl;
	
}

//vectorからpcdへ変換
void PCL::_convert_array_to_pcd(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	if (points.size() == 0) return;

	if (pcd_type_ == PCL_XYZ) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
		//std::cout << "array to pcd" << std::endl;
		cloud_tmp->width = number_of_points;
		cloud_tmp->height = 1;
		cloud_tmp->is_dense = false;
		cloud_tmp->points.resize(cloud_tmp->width * cloud_tmp->height);

		float px, py, pz;
		for (int i = 0; i < number_of_points; i++) {
			px = points[3 * i + 0];
			py = points[3 * i + 1];
			pz = points[3 * i + 2];
			cloud_tmp->points[i] = pcl::PointXYZ(px, py, pz);
		}
		//std::cout << "px"<<px << std::endl;
		cloud_xyz_ = cloud_tmp;
		//std::cout << cloud->size() << std::endl;
	}
	else if(pcd_type_ == PCL_XYZRGB){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
		//std::cout << "array to pcd" << std::endl;
		cloud_tmp->width = number_of_points;
		cloud_tmp->height = 1;
		cloud_tmp->is_dense = false;
		cloud_tmp->points.resize(cloud_tmp->width * cloud_tmp->height);

		float px, py, pz;
		int r, g, b;
		for (int i = 0; i < number_of_points; i++) {
			px = points[3 * i + 0];
			py = points[3 * i + 1];
			pz = points[3 * i + 2];
			r = color[3 * i + 0];
			g = color[3 * i + 1];
			b = color[3 * i + 2];
			cloud_tmp->points[i] = pcl::PointXYZRGB(px, py, pz,r,g,b);
		}
		//std::cout << "px"<<px << std::endl;
		cloud_xyzrgb_ = cloud_tmp;
		//std::cout << cloud->size() << std::endl;
	}
	else if (pcd_type_ == PCL_XYZRGBA) {
		cloud_xyzrgba_->clear();

		float px, py, pz;
		int r, g, b;
		int a;
		for (int i = 0; i < number_of_points; i++) {
			px = points[3 * i + 0];
			py = points[3 * i + 1];
			pz = points[3 * i + 2];
			r = color[3 * i + 0];
			g = color[3 * i + 1];
			b = color[3 * i + 2];
			a = 0;
			cloud_xyzrgba_->push_back(pcl::PointXYZRGBA(px, py, pz, r, g, b, a));
		}
	}
}

//pcdからvectorへ変換
void PCL::_convert_pcd_to_array(int& number_of_points, std::vector<float>& points, std::vector<int>& color) {
	if (cloud_xyzrgba_->size() == 0) return;

	//std::cout << "pcd to array" << std::endl;
	points.clear();
	color.clear();

	if (pcd_type_ == PCL_XYZ) {

		for (int i = 0; i < cloud_xyz_->size(); i++) {
			points.push_back(cloud_xyz_->points[i].x);
			points.push_back(cloud_xyz_->points[i].y);
			points.push_back(cloud_xyz_->points[i].z);
		}
		number_of_points = points.size() / 3;
		//*color = *color;
		//std::cout<< points.size()/3 <<std::endl;
	}
	else if(pcd_type_ == PCL_XYZRGB){
		for (int i = 0; i < cloud_xyzrgb_->size(); i++) {
			points.push_back(cloud_xyzrgb_->points[i].x);
			points.push_back(cloud_xyzrgb_->points[i].y);
			points.push_back(cloud_xyzrgb_->points[i].z);
			color.push_back(cloud_xyzrgb_->points[i].r);
			color.push_back(cloud_xyzrgb_->points[i].g);
			color.push_back(cloud_xyzrgb_->points[i].b);
		}

		number_of_points = points.size() / 3;
	}
	else if (pcd_type_ == PCL_XYZRGBA) {
		for(const auto& p: *cloud_xyzrgba_){
			points.push_back(p.x);
			points.push_back(p.y);
			points.push_back(p.z);
			color.push_back(p.r);
			color.push_back(p.g);
			color.push_back(p.b);
		}

		number_of_points = points.size() / 3;
		//std::cout << "                               pcd to array  " << number_of_points << std::endl;
	}
}



//octreeによるデータ圧縮
void PCL::_compress() {
	double a = clock();
	std::stringstream compressedData_;

	if (pcd_type_ == PCL_XYZ) {
		if (cloud_xyz_->size() > 0) {
			pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression_xyz_(pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR, true);
			octreeCompression_xyz_.encodePointCloud(cloud_xyz_, compressedData_);
			octreeCompression_xyz_.decodePointCloud(compressedData_, cloud_xyz_);
			//frame_++;
		}
	}
	else {
		if (cloud_xyzrgb_->size() > 0) {
			pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> octreeCompression_xyzrgb_(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR, false);
			octreeCompression_xyzrgb_.encodePointCloud(cloud_xyzrgb_, compressedData_);
			octreeCompression_xyzrgb_.decodePointCloud(compressedData_, cloud_xyzrgb_);
			//frame_++;
		}
	}
	std::cout<< "time for compression   "<<double(clock()-a)/1000 <<std::endl;
}

void PCL::_detect_change(std::vector<int>& color) {
	if (pcd_type_ == PCL_XYZ) {
		if (cloud_xyz_->size() > 0 && cloud_previous_xyz_ != nullptr) {
			float resolution = 0.02;
			pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_xyz_(resolution);
			std::vector<int> newPointIdxVector;
			////////////////////    appeared
			octree_xyz_.setInputCloud(cloud_previous_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			octree_xyz_.switchBuffers();
			octree_xyz_.setInputCloud(cloud_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			newPointIdxVector.clear();
			octree_xyz_.getPointIndicesFromNewVoxels(newPointIdxVector);
			appeared_points_.clear();
			for (std::size_t i = 0; i < newPointIdxVector.size(); ++i) {
				color[3 * newPointIdxVector[i] + 0] = 0;
				color[3 * newPointIdxVector[i] + 1] = 255 - color[3 * newPointIdxVector[i] + 1];
				color[3 * newPointIdxVector[i] + 2] = 255 - color[3 * newPointIdxVector[i] + 2];
				appeared_points_.push_back((*cloud_xyz_)[newPointIdxVector[i]].x);
				appeared_points_.push_back((*cloud_xyz_)[newPointIdxVector[i]].y);
				appeared_points_.push_back((*cloud_xyz_)[newPointIdxVector[i]].z);
			}
			////////////////////  disappeared
			octree_xyz_.setInputCloud(cloud_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			octree_xyz_.switchBuffers();
			// Add points from cloudB to octree_xyz_
			octree_xyz_.setInputCloud(cloud_previous_xyz_);
			octree_xyz_.addPointsFromInputCloud();
			newPointIdxVector.clear();
			// Get vector of point indices from octree_xyz_ voxels which did not exist in previous buffer
			octree_xyz_.getPointIndicesFromNewVoxels(newPointIdxVector);
			disappeared_points_.clear();
			for (std::size_t i = 0; i < newPointIdxVector.size(); ++i) {
				//color[3 * newPointIdxVector[i] + 0] = 255 - color[3 * newPointIdxVector[i] + 0];
				//color[3 * newPointIdxVector[i] + 1] = 255 - color[3 * newPointIdxVector[i] + 1];
				//color[3 * newPointIdxVector[i] + 2] = 255 - color[3 * newPointIdxVector[i] + 2];
			}
		}
		cloud_previous_xyz_ = cloud_xyz_;
	}
}

void PCL::_remove_plane() {
	bool loop = true;
	while (loop) {
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.002);

		seg.setInputCloud(cloud_xyzrgba_);
		seg.segment(*inliers, *coefficients);

		std::cout << inliers->indices.size() << std::endl;
		if(inliers->indices.size() > 0.3*cloud_xyzrgba_->size()) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyz_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
			pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
			extract.setInputCloud(cloud_xyzrgba_);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*cloud_xyz_filtered);
			cloud_xyzrgba_->resize(0);
			cloud_xyzrgba_ = cloud_xyz_filtered;

			loop = true;
		}
		else {
			loop = false;

		}
	}
}

void PCL::_detect_contact_with_particlefilter() {

	if (!track_inited_) {

		ParticleT bin_size;
		bin_size.x = 0.1f;
		bin_size.y = 0.1f;
		bin_size.z = 0.1f;
		bin_size.roll = 0.1f;
		bin_size.pitch = 0.1f;
		bin_size.yaw = 0.1f;

		//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
		tracker->setMaximumParticleNum(1000);
		tracker->setDelta(0.99);
		tracker->setEpsilon(0.15);
		tracker->setBinSize(bin_size);

		//Set all parameters for  ParticleFilter
		tracker_ = tracker;
		tracker_->setTrans(Eigen::Affine3f::Identity());
		std::vector<double> default_step_covariance = std::vector<double>{ 0.015 * 0.015,0.015 * 0.015, 0.015 * 0.015, 0.015 * 0.015 * 40.0,0.015 * 0.015 * 40.0,0.015 * 0.015 * 40.0 }; // 1 * 6 vector
		std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001); // 1 * 6 vector
		std::vector<double> default_initial_mean = std::vector<double>{ -0.15 * 1., -0.25 * 1., 0.27 * 1., 0., 0., 0. };// 1 * 6 vector xyzrpy
		//std::vector<double> default_initial_mean = std::vector<double>{ -0.16 * 1., -0.26 * 1., 0.22 * 0., 0., 0., 0. };// 1 * 6 vector xyzrpy
		tracker_->setStepNoiseCovariance(default_step_covariance);
		tracker_->setInitialNoiseCovariance(initial_noise_covariance);
		tracker_->setInitialNoiseMean(default_initial_mean);
		tracker_->setIterationNum(3);
		tracker_->setParticleNum(1000);
		tracker_->setResampleLikelihoodThr(0.00);
		tracker_->setUseNormal(false);

		//PL(1)


		//Setup coherence object for tracking
		coherence->addPointCoherence(hsv_coherence);
		coherence->addPointCoherence(distance_coherence);
		coherence->setSearchMethod(search);
		coherence->setMaximumDistance(0.01);
		tracker_->setCloudCoherence(coherence);

		trans = contact_detector_->init_for_particlefilter();
		tracker_->setTrans(trans);
		tracker_->setReferenceCloud(contact_detector_->get_cloud_target());

		track_inited_ = true;

	}

	std::lock_guard<std::mutex> lock(mtx_);
	cloud_pass_.reset(new Cloud);
	cloud_pass_downsampled_.reset(new Cloud);
	gridSampleApprox(cloud_xyzrgba_, *cloud_pass_downsampled_, downsampling_grid_size_);


	if (counter < 2) return;

	PL(cloud_pass_downsampled_->size())
	PL(cloud_xyzrgba_->size())

	//Track the object
	tracker_->setInputCloud(cloud_xyzrgba_);
	tracker_->compute();
	new_cloud_ = true;


	ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
	//std::cout << tracker_->getParticles() << std::endl;
	if (tracker_->getParticles() && new_cloud_ || false)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
		for (const auto& particle : *particles)
		{
			pcl::PointXYZRGBA point;
			point.x = particle.x;
			point.y = particle.y;
			point.z = particle.z;
			particle_cloud->push_back(point);
			//cloud_xyzrgba_->push_back(point);
		}

		
		ParticleXYZRPY result = tracker_->getResult();
		transformation = tracker_->toEigenMatrix(result);

		contact_detector_->transform_init(transformation);

		contact_detector_->remove_and_detect(cloud_xyzrgba_);


		for (const auto& particle : *particles)
		{
			pcl::PointXYZRGBA point;
			point.x = particle.x;
			point.y = particle.y;
			point.z = particle.z;
			point.r = 255;
			point.g = 255;
			cloud_xyzrgba_->push_back(point);
		}

	new_cloud_ = false;
	

}

double PCL::_computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (std::size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

void PCL::_recognite() {
	show_keypoints_=(false);
	show_correspondences_=(false);
	use_cloud_resolution_=(true);
	use_hough_=(true);
	model_ss_=(0.01f);
	scene_ss_=(0.03f);
	rf_rad_=(0.015f);
	descr_rad_=(0.02f);
	cg_size_=(0.01f);
	cg_thresh_=(5.0f);

	model = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	model_keypoints = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	scene = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	scene_keypoints = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
	model_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
	scene_normals = pcl::PointCloud<NormalType>::Ptr(new pcl::PointCloud<NormalType>());
	model_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType>());
	scene_descriptors = pcl::PointCloud<DescriptorType>::Ptr(new pcl::PointCloud<DescriptorType>());

	if (pcl::io::loadPCDFile("src/pointpkg/pcd/MS_xyzrgba_common/model2.pcd", *model) < 0) {
		std::cout << "Error loading model cloud." << std::endl;
	}


	if (pcl::io::loadPCDFile("src/pointpkg/pcd/MS_xyzrgba3/101.pcd", *scene) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		return;
	}

//  Set up resolution invariance
//
	if (use_cloud_resolution_)
	{
		float resolution = static_cast<float> (_computeCloudResolution(model));
		if (resolution != 0.0f)
		{
			model_ss_ *= resolution;
			scene_ss_ *= resolution;
			rf_rad_ *= resolution;
			descr_rad_ *= resolution;
			cg_size_ *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
	}

	
	//  Compute Normals
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);


	

//  Downsample Clouds to Extract keypoints


	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

	
	//
//  Compute Descriptor for keypoints
//
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);
	
	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);

	std::cout << scene->size() << std::endl;

	//return;


//  Find Model-Scene Correspondences with KdTree

	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	//  Using Hough3D
	if (use_hough_)
	{
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
		pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);

		rf_est.setInputCloud(model_keypoints);
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model);
		rf_est.compute(*model_rf);

		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene);
		rf_est.compute(*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);

		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//clusterer.cluster (clustered_corrs);
		clusterer.recognize(rototranslations, clustered_corrs);
	}
	else // Using GeometricConsistency
	{
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize(rototranslations, clustered_corrs);
	}

	//
	//  Output results
	//
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (std::size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}

	//
//  Visualization
//
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

	if (show_correspondences_ || show_keypoints_)
	{
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}

	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}

	for (std::size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		if (show_correspondences_)
		{
			for (std::size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
			}
		}
	}

	while (!viewer.wasStopped())
	{
		//std::cout << "viewer loop" << std::endl;
		viewer.spinOnce();
	}
}

//Filter along a specified dimension
void PCL::_filterPassThrough(){
	if (cloud_xyzrgba_->size() == 0) return;
	if (!enable_filterPathThrough) return;

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pass.setKeepOrganized(true);

	pass.setFilterFieldName("x");
	pass.setFilterLimits(threshold_[0], threshold_[1]);
	pass.setInputCloud(cloud_xyzrgba_);
	pass.filter(*cloud_xyzrgba_);

	pass.setFilterFieldName("y");
	pass.setFilterLimits(threshold_[2], threshold_[3]);
	pass.setInputCloud(cloud_xyzrgba_);
	pass.filter(*cloud_xyzrgba_);

	pass.setFilterFieldName("z");
	pass.setFilterLimits(threshold_[4], threshold_[5]);
	pass.setInputCloud(cloud_xyzrgba_);
	pass.filter(*cloud_xyzrgba_);
	//cloud_xyzrgba_ = result;
}

void PCL::gridSampleApprox(const CloudConstPtr& cloud, Cloud& result, double leaf_size)
{
	//pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
	grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
}

//Draw the current particles
bool PCL::drawParticles(pcl::visualization::PCLVisualizer& viz)
{
	ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
	if (particles && new_cloud_)
	{
		//Set pointCloud with particle's points
		pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		for (const auto& particle : *particles)
		{
			pcl::PointXYZ point;

			point.x = particle.x;
			point.y = particle.y;
			point.z = particle.z;
			particle_cloud->push_back(point);
		}

		//Draw red particles 
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(particle_cloud, 250, 99, 71);

			if (!viz.updatePointCloud(particle_cloud, red_color, "particle cloud"))
				viz.addPointCloud(particle_cloud, red_color, "particle cloud");
		}
		return true;
	}
	else
	{
		return false;
	}
}

//Draw model reference point cloud
void PCL::drawResult()
{
	viewer->showCloud(cloud_xyzrgba_);
	

}


void PCL::_edit() {
	//0.0405
	//0.0825
	std::cout << "edit" << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for (const auto& p : *cloud_xyzrgba_) {
		if (p.y < 0.) tmp->push_back(p);
		ZZ = std::max(ZZ, p.z);
	}
	cloud_xyzrgba_ = tmp;
	std::cout << ZZ << std::endl;
}

void PCL::_noise_filter() {

}


void PCL::_RadiusSearch() {
	/*
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;

	kdtree.setInputCloud(cloud_xyzrgba_);

	pcl::PointXYZRGBA searchPoint;
	//searchPoint

	if(_RadiusSearch_is_first){
	target_cloud.reset(new Cloud());
	if (pcl::io::loadPCDFile(target, *target_cloud) == -1) {
		std::cout << "pcd file not found" << std::endl;
		exit(-1);
	}
	*/
}


void PCL::_transform() {
	if (cloud_xyzrgba_->size() == 0) return;

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << transformation_parameters_[0], transformation_parameters_[1], transformation_parameters_[2];
	//transform.rotate(Eigen::AngleAxisf(transformation_parameters_[3], Eigen::Vector3f::UnitX()));
	//transform.rotate(Eigen::AngleAxisf(transformation_parameters_[4], Eigen::Vector3f::UnitY()));
	//transform.rotate(Eigen::AngleAxisf(transformation_parameters_[5], Eigen::Vector3f::UnitZ()));

	Eigen::Affine3f rotX = Eigen::Affine3f::Identity();
	rotX.rotate(Eigen::AngleAxisf(transformation_parameters_[3], Eigen::Vector3f::UnitX()));

	Eigen::Affine3f rotY = Eigen::Affine3f::Identity();
	rotY.rotate(Eigen::AngleAxisf(transformation_parameters_[4], Eigen::Vector3f::UnitY()));

	Eigen::Affine3f rotZ = Eigen::Affine3f::Identity();
	rotZ.rotate(Eigen::AngleAxisf(transformation_parameters_[5], Eigen::Vector3f::UnitZ()));

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>());

	pcl::transformPointCloud(*cloud_xyzrgba_, *cloud_tmp, rotX);
	pcl::transformPointCloud(*cloud_tmp, *cloud_xyzrgba_, rotY);
	pcl::transformPointCloud(*cloud_xyzrgba_, *cloud_tmp, rotZ);
	pcl::transformPointCloud(*cloud_tmp, *cloud_xyzrgba_, transform);


	//pcl::transformPointCloud(*cloud_xyzrgba_, *cloud_tmp, transform);
	//pcl::transformPointCloud(*cloud_tmp, *cloud_xyzrgba_, rotX);
	//pcl::transformPointCloud(*cloud_xyzrgba_, *cloud_tmp, rotY);
	//pcl::transformPointCloud(*cloud_tmp, *cloud_xyzrgba_, rotZ);

	//cloud_xyzrgba_ = cloud_tmp;

	//std::cout << transform.matrix() << std::endl;

}

void PCL::_detect_contact_with_coppeliasim(){
	if (cloud_xyzrgba_->size() == 0) return;
	////絶対座標原点に戻す
	std::vector<float> RMT = coppeliasim_interface_->get_remover_transform();
	contact_detector_->transform_init(&RMT[0], &RMT[3], true);

	std::vector<float> LT = coppeliasim_interface_->get_L_tip_transform();
	contact_detector_->transform_rotated(&LT[0], &LT[3], false);

	contact_detector_->remove_and_detect(cloud_xyzrgba_);

}




void PCL::_read_pcd_file(std::string folder, int& number_of_points, std::vector<int>& color) {
	std::string file;
	if (folder.find(".pcd") != std::string::npos) file = folder;
	else {
		file = folder + std::to_string(cnt_read_);
		//file += std::to_string(cnt_read_);
		file += ".pcd";
		//std::cout << "qwert" << std::endl;
	}

	if (pcd_type_ == PCL_XYZ) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud_tmp) == -1) //* load the file
		{
			//PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			cnt_read_ = 0;
		}
		else {
			cloud_xyz_ = cloud_tmp;
			number_of_points = cloud_xyz_->width * cloud_xyz_->height;
			cnt_read_++;
			color_.clear();
			for (int i = 0; i < cloud_xyz_->width * cloud_xyz_->height; i++) {
				color_.push_back(255);
				color_.push_back(0);
				color_.push_back(0);
			}
			color = color_;
		}
	}
	else if(pcd_type_ == PCL_XYZRGB) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file, *cloud_tmp) == -1) //* load the file
		{
			//PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			cnt_read_ = 0;
		}
		else {
			cloud_xyzrgb_ = cloud_tmp;
			number_of_points = cloud_xyzrgb_->width * cloud_xyzrgb_->height;
			cnt_read_++;

			color = color_;
		}
	}
	else if (pcd_type_ == PCL_XYZRGBA) {
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
		//cloud_xyzrgba_->clear();
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file, *cloud_tmp) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			cnt_read_ = 0;
		}
		else {
			
			for (const auto& p : *cloud_tmp) {	cloud_xyzrgba_->push_back(p);}
			//cloud_xyzrgba_ = cloud_tmp;
			number_of_points = cloud_xyzrgba_->width * cloud_xyzrgba_->height;
			cnt_read_++;
			//std::cout << "read" << endl;
		}
	}
}

void PCL::_write_pcd_file(std::string folder) {
	if (cloud_xyzrgba_->size() == 0) return;
	
	if (pcd_type_ == PCL_XYZ && cloud_xyz_->size() > 0) {
		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_xyz_);
		cnt_write_++;
	}
	if (pcd_type_ == PCL_XYZRGB && cloud_xyzrgb_->size() > 0) {
		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_xyzrgb_);
		cnt_write_++;
	}
	if (pcd_type_ == PCL_XYZRGBA && cloud_xyzrgba_->size() > 0) {
		pcl::io::savePCDFileASCII(folder + std::to_string(cnt_write_) + ".pcd", *cloud_xyzrgba_);
		cnt_write_++;
	}
	//std::cerr << "Saved " << cloud_xyz_->size() << " data points to test_pcd.pcd." << std::endl;
}


void PCL::_load_parameters(std::string file_path) {

	ifstream ifs(file_path);
	std::stringstream ss;
	std::string line;
	//std::getline(ifs, line);

	std::string parameter_name;
	std::string value;
	while (getline(ifs, line)) {
		std::istringstream i_stream(line);
		std::getline(i_stream, parameter_name, ' ');
		std::getline(i_stream, value);
		//std::cout << parameter_name << " " << value << std::endl;

		if ( parameter_name=="trans_x")   transformation_parameters_[0] = stof(value);
		if ( parameter_name=="trans_y")   transformation_parameters_[1] = stof(value);
		if ( parameter_name=="trans_z")   transformation_parameters_[2] = stof(value);
		if ( parameter_name=="trans_ax")   transformation_parameters_[3] = stof(value);
		if ( parameter_name=="trans_ay")   transformation_parameters_[4] = stof(value);
		if ( parameter_name=="trans_az")   transformation_parameters_[5] = stof(value);

		if ( parameter_name=="thresh_min_x")   threshold_[0] = stof(value);
		if ( parameter_name=="thresh_max_x")   threshold_[1] = stof(value);
		if ( parameter_name=="thresh_min_y")   threshold_[2] = stof(value);
		if ( parameter_name=="thresh_max_y")   threshold_[3] = stof(value);
		if ( parameter_name=="thresh_min_z")   threshold_[4] = stof(value);
		if ( parameter_name=="thresh_max_z")   threshold_[5] = stof(value);

	}

	std::cout << "pcl; parameters loaded" << std::endl;

}

void PCL::_save_parameters(std::string file_path) {

	ofstream ofs(file_path);
	ofs << std::fixed<<std::setprecision(6);


	for (int i = 0; i < 6; i++) if (abs(transformation_parameters_[i]) < 0.00001) transformation_parameters_[i] = 0.0;
	ofs << "trans_x "<<transformation_parameters_[0]<<endl;
	ofs << "trans_y "<<transformation_parameters_[1]<<endl;
	ofs << "trans_z "<<transformation_parameters_[2]<<endl;
	ofs << "trans_ax "<<transformation_parameters_[3]<<endl;
	ofs << "trans_ay "<<transformation_parameters_[4]<<endl;
	ofs << "trans_az "<<transformation_parameters_[5]<<endl;

	for (int i = 0; i < 6; i++) if (abs(threshold_[i]) < 0.00001) threshold_[i] = 0.0;
	ofs << "thresh_min_x "<<threshold_[0]<<endl;
	ofs << "thresh_max_x "<<threshold_[1]<<endl;
	ofs << "thresh_min_y "<<threshold_[2]<<endl;
	ofs << "thresh_max_y "<<threshold_[3]<<endl;
	ofs << "thresh_min_z "<<threshold_[4]<<endl;
	ofs << "thresh_max_z "<<threshold_[5]<<endl;
	ofs.close();

}







ContactDetector::ContactDetector(std::string pcd_target, std::string pcd_mask="", std::string pcd_tip="")
	//:	octree_(0.005)
{
	cloud_target_init_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcl::io::loadPCDFile(pcd_target, *cloud_target_init_) == -1) {
		std::cout << "remover::remover; pcd target file not found" << std::endl;
		exit(-1);
	}
	cloud_mask_init_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcd_mask!="" && pcl::io::loadPCDFile(pcd_mask, *cloud_mask_init_) == -1) {
		std::cout << "remover::remover; pcd mask file not found" << std::endl;
		exit(-1);
	}
	cloud_tip_init_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
	if (pcd_tip!="" && pcl::io::loadPCDFile(pcd_tip, *cloud_tip_init_) == -1) {
		std::cout << "remover::remover; pcd tip file not found" << std::endl;
		exit(-1);
	}
}


void ContactDetector::transform_init(float* pos, float* ori, bool inverse) {
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << pos[0], pos[1], +pos[2];
	transform.rotate(Eigen::AngleAxisf(ori[0], Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(ori[1], Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(ori[2], Eigen::Vector3f::UnitZ()));

	if (inverse == true) transform = transform.inverse();

	cloud_target_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
	cloud_mask_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
	cloud_tip_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
}


void ContactDetector::transform_init(const Eigen::Affine3f& transform) {
	//PL("aaa")
	//PL(cloud_target_init_->size())
	cloud_target_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_target_init_, *cloud_target_transformed_, transform);
	cloud_mask_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_mask_init_, *cloud_mask_transformed_, transform);
	cloud_tip_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud(*cloud_tip_init_, *cloud_tip_transformed_, transform);
}


void ContactDetector::transform_rotated(float* pos, float* ori, bool inverse) {
	//std::cout << "rotated" << std::endl;
	//for (int i = 0; i < 6; i++) std::cout << pos[i] << " ";
	//std::cout << endl;
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << pos[0], pos[1], pos[2];
	//std::cout << "transform rotated target" << transform.matrix() << std::endl;
	transform.rotate(Eigen::AngleAxisf(ori[0], Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(ori[1], Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(ori[2], Eigen::Vector3f::UnitZ()));
	//std::cout << "transform rotated target" << transform.matrix() << std::endl;
	if (inverse == true) transform = transform.inverse();

	//pcl::PointCloud<pcl::PointXYZRGBA> cloud_tmp;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>());
	cloud_tmp = cloud_target_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_target_transformed_, transform);
	cloud_tmp = cloud_mask_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_mask_transformed_, transform);
	cloud_tmp = cloud_tip_transformed_;
	pcl::transformPointCloud(*cloud_tmp, *cloud_tip_transformed_, transform);
}

void ContactDetector::remove_and_detect(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_xyzrgba) {
	//return;

	//cloud_ と resultの一致検出
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> octree(0.005);
	octree.setInputCloud(cloud_mask_transformed_);
	octree.addPointsFromInputCloud();
	octree.switchBuffers();
	octree.setInputCloud(cloud_xyzrgba);
	octree.addPointsFromInputCloud();


	std::vector<int> newPointIdxVector;
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	for (int i = 0; i < newPointIdxVector.size(); i++) {
		inliers->indices.push_back(newPointIdxVector[i]);
	}
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr consensus(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr different(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	//extract.setKeepOrganized(true);
	extract.setInputCloud(cloud_xyzrgba);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*different);
	extract.setNegative(true);//red
	extract.filter(*consensus);

	std::cout << cloud_xyzrgba->size() << " " << different->size() << " " << consensus->size() <<" "<< newPointIdxVector.size()<< std::endl;

	cloud_xyzrgba = different;
	//*cloud_xyzrgba = *cloud_mask_transformed_;
	//std::cout << cloud_xyzrgba->size() << std::endl;


	//cloud_xyzrgba = cloud_mask_transformed_;

	pcl::PointXYZRGBA searchPoint = (*cloud_tip_transformed_)[0];
	searchPoint.r = 255;
	searchPoint.g = 0;
	searchPoint.b = 0; 

	if (cloud_xyzrgba->size() > 0) {
		pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
		kdtree.setInputCloud(cloud_xyzrgba);



		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquareDistance;

		float R = 0.01;
		int num = 200;
		int n = kdtree.radiusSearch(searchPoint, R, pointIdxRadiusSearch, pointRadiusSquareDistance);
		std::cout << "number of nearest point " << n << std::endl;
		if (n > num) for (int i = 0; i < int(std::log2(n-num)); i++) std::cout << "##";
		std::cout << std::endl;
	}
	
	cloud_xyzrgba->push_back(searchPoint);
	

	for (auto& point : *consensus) {
		//point.r = 0;// -cloud.r;
		//point.g = 0;// -cloud.g;
		point.b = 255;// -cloud.b;
		cloud_xyzrgba->push_back(point);
	}

}

Eigen::Affine3f ContactDetector::init_for_particlefilter() {
	Eigen::Vector4f c;
	pcl::compute3DCentroid<pcl::PointXYZRGBA>(*cloud_target_init_, c);
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);

	cloud_target_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_target_init_, *cloud_target_transformed_, trans.inverse());
	//gridSampleApprox(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
	*cloud_target_init_ = *cloud_target_transformed_;

	cloud_mask_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_mask_init_, *cloud_mask_transformed_, trans.inverse());
	*cloud_mask_init_ = *cloud_mask_transformed_;

	cloud_tip_transformed_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_tip_init_, *cloud_tip_transformed_, trans.inverse());
	*cloud_tip_init_ = *cloud_tip_transformed_;

	return  trans;
}
