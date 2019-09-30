/*
 * multiPC_ICP.cpp
 *
 *  Created on: Mar 6, 2019
 *      Author: idf
 */
#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
// pcl: normal estimation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
// pcl: for voxel downsampling
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/surface/mls.h>
#include <pthread.h>
#include <pcl/registration/icp.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <chrono>
#include <math.h>

//typedef pcl::Normal NormalType;
typedef pcl::PointNormal NormalType;
typedef pcl::PointXYZ PointType;

pcl::PointCloud<PointType>::Ptr cloud1(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud3(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr cloud1_init(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr cloud2_init(new pcl::PointCloud<PointType>());
//pcl::PointCloud<PointType>::Ptr cloud3_init(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_merged(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloud_filtered(
		new pcl::PointCloud<PointType>());
pcl::PointCloud<NormalType>::Ptr model(new pcl::PointCloud<NormalType>());
pcl::PointCloud<NormalType>::Ptr adjusted_model(
		new pcl::PointCloud<NormalType>());
pcl::PointCloud<NormalType>::Ptr initial_model(
		new pcl::PointCloud<NormalType>());
pcl::PointCloud<PointType>::Ptr model_won(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr adjusted_model_won(
		new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr initial_model_won(
		new pcl::PointCloud<PointType>());

Eigen::Matrix4f trasl, trtotal, transf_total;
Eigen::MatrixXf trot(4, 4), tr_msg(4, 4);
Eigen::Matrix3f m;

//sensor_msgs::PointCloud2 input1;
//sensor_msgs::PointCloud2 input2;
//sensor_msgs::PointCloud2 input3;
sensor_msgs::PointCloud2 cloud_in1;
sensor_msgs::PointCloud2 cloud_in2;
sensor_msgs::PointCloud2 cloud_in3;
sensor_msgs::PointCloud2 cloud_inter1;
sensor_msgs::PointCloud2 cloud_inter2;
sensor_msgs::PointCloud2 cloud_inter3;
//Eigen::Matrix4f eigen_transform, eigen_transform2, eigen_transform3, eigen_transform4;
Eigen::Matrix4f eigen_transform4;
tf::StampedTransform transform1;
tf::StampedTransform transform2;
tf::StampedTransform transform3;
tf::StampedTransform transform4;

sensor_msgs::PointCloud2 output1;

sensor_msgs::PointCloud2 output_merged;
geometry_msgs::Transform tf_model;
geometry_msgs::PoseStamped tf_hololens;

std_msgs::Float32 fitness_score;
std_msgs::Int32 do_tr;

ros::Publisher pub;
ros::Publisher pub_m;
ros::Publisher pub_transform;
ros::Publisher pub_transform_hololens;
ros::Publisher pub_fit_score;
ros::Publisher pub_do_tr;

ros::Subscriber *sub1;
ros::Subscriber *sub2;
ros::Subscriber *sub3;

tf::TransformListener *listener1;
tf::TransformListener *listener2;
tf::TransformListener *listener3;
tf::TransformListener *listener4;

int k = 0;
int c = 0;
float prev_fit = 1, fit;
int prev_it, iterac;

bool cam1_done = false;
bool cam2_done = false;
bool cam3_done = false;
int cam_now = 1;

//bool hard_code = false;

bool cam_callbacks = true;
unsigned int seq = 0;

ros::Time old_icp;


void filtered_pointcloud(sensor_msgs::PointCloud2 cloud_in, tf::StampedTransform transform, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, pcl::PointCloud<PointType>::Ptr &cloud) {

	sensor_msgs::PointCloud2 ros_cloud;
	Eigen::Matrix4f eigen_transform;

	// Convert the ROS sensor_msgs/PointCloud2 data to PCLPointCloud2
	pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(cloud_in, *pcl_cloud);

//	// Original cloud size
//	std::cerr << "raw PointCloud before filtering: "
//			<< pcl_cloud->width * pcl_cloud->height << " data points."
//			<< std::endl;

	// Perform actual downsampling filtering
	pcl::PCLPointCloud2ConstPtr pcl_cloud_ptr(pcl_cloud);
	pcl::PCLPointCloud2 pcl_cloud_downsampled;

	pcl::VoxelGrid<pcl::PCLPointCloud2> voxelgrid;
	voxelgrid.setInputCloud(pcl_cloud_ptr);
	// choose downsample, larger leaf size numbers, less points.
	// fully dependant on the actual cloud we are dealing with.
	// it should really be a config parameter.
	voxelgrid.setLeafSize(0.01, 0.01, 0.01);

	voxelgrid.filter(pcl_cloud_downsampled);

	// For some reason I need to go from pcl::PCLPointCloud2 to pcl::PointCloud<PointXYZ> to
	// use the code to filter NaN...??
	pcl::PointCloud<PointType>::Ptr pcl_cloud_pre(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr pcl_cloud_clean (new pcl::PointCloud<PointType> ());
	pcl::fromPCLPointCloud2(pcl_cloud_downsampled, *pcl_cloud_pre);
//	pcl::fromPCLPointCloud2(*pcl_cloud, *pcl_cloud_pre);

	// Filter NaNs from the cloud, otherwise mls fails
	for (size_t i = 0; i < pcl_cloud_pre->size(); ++i)
		if (pcl_isfinite(pcl_cloud_pre->points[i].x))
			pcl_cloud_clean->push_back(pcl_cloud_pre->points[i]);
	//			else
	//				std::cout << "NaN";
	pcl_cloud_clean->header = pcl_cloud->header;
	pcl_cloud_clean->height = 1;
	pcl_cloud_clean->width = static_cast<uint32_t>(pcl_cloud_clean->size());
	pcl_cloud_clean->is_dense = false;

//	// Clean cloud size
//	//	std::cerr << pcl_cloud_clean->width << endl;
//	//	std::cerr << pcl_cloud_clean->height << endl;
//	std::cerr << "clean PointCloud : "
//			<< pcl_cloud_clean->width * pcl_cloud_clean->height
//			<< " data points." << std::endl;

	// Convert original ROS PointCloud to PCL PointCloud
	//pcl::fromROSMsg(cloud_in, *cloud_init);
	// Define local passthrough filter to avoid distorted 3D points
	pcl::PassThrough<PointType> pass_cloud;
	//  pass_cloud.setInputCloud (cloud_init);
	pass_cloud.setInputCloud(pcl_cloud_clean);
	pass_cloud.setFilterFieldName("z");
	pass_cloud.setFilterLimits(z_min, z_max);
	//  pass_cloud.filter (*cloud_init);
	pass_cloud.filter(*pcl_cloud_clean);
	// Convert filtered pcl pointcloud to ros pointcloud
	//pcl::toROSMsg(*cloud_init, cloud_inter);
//	pcl::toROSMsg(*pcl_cloud_clean, cloud_inter);
//	cloud_inter.header = cloud_in.header;
//	pcl_ros::transformAsMatrix(transform, eigen_transform);
//	pcl_ros::transformPointCloud(eigen_transform, cloud_inter, input);
	pcl::toROSMsg(*pcl_cloud_clean, ros_cloud);
	ros_cloud.header = cloud_in.header;
	pcl_ros::transformAsMatrix(transform, eigen_transform);
	pcl_ros::transformPointCloud(eigen_transform, ros_cloud, ros_cloud);
	ros_cloud.header.frame_id = "/cell_link";
	// Convert transformed ros pointcloud to pcl pointcloud
	pcl::fromROSMsg(ros_cloud, *cloud);
	// Define global passthrough filter to remove outlier points
	pass_cloud.setInputCloud(cloud);
	pass_cloud.setFilterFieldName("x");
	pass_cloud.setFilterLimits(x_min, x_max);
	pass_cloud.filter(*cloud);
	pass_cloud.setInputCloud(cloud);
	pass_cloud.setFilterFieldName("y");
	pass_cloud.setFilterLimits(y_min, y_max);
	pass_cloud.filter(*cloud);

}

void cloud_cb1(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1) {
	cloud_in1 = *cloud_msg1;
	//cam1_done = true;
	// Convert and filter the ROS sensor_msgs/PointCloud2 data to PCLPointCloud2
	filtered_pointcloud(cloud_in1, transform1, -0.5, 1.0, -1.0, 0.05, 0.5, 1.8, cloud1);

	// Merge all three clouds
	*cloud_merged  = *cloud1;

	sub1->shutdown();
	cam_now = 2;
}

void cloud_cb2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg2) {
	cloud_in2 = *cloud_msg2;
	//cam2_done = true;
	// Convert and filter the ROS sensor_msgs/PointCloud2 data to PCLPointCloud2
	filtered_pointcloud(cloud_in2, transform2, -1.0, 0.5, -1.0, 0.05, 0.5, 1.8, cloud2);

	// Merge all three clouds
	*cloud_merged += *cloud2;

	sub2->shutdown();
	cam_now = 3;
}

void cloud_cb3(const sensor_msgs::PointCloud2ConstPtr& cloud_msg3) {
	cloud_in3 = *cloud_msg3;
	//cam3_done = true;
	// Convert and filter the ROS sensor_msgs/PointCloud2 data to PCLPointCloud2
	filtered_pointcloud(cloud_in3, transform3, -1.0, 1.0, -0.1,  1.0, 0.5, 1.8, cloud3);

	// Merge all three clouds
	*cloud_merged += *cloud3;

	sub3->shutdown();
	cam_now = 1;
	cam_callbacks = false;
}

int main(int argc, char** argv) {

	// Initialize ROS
	ros::init(argc, argv, "multiPC_ICP_multicam_new");
	ros::NodeHandle nh;

	pub = nh.advertise<sensor_msgs::PointCloud2>("output1", 1);
	pub_m = nh.advertise<sensor_msgs::PointCloud2>("output_merged_2", 1);
	pub_transform = nh.advertise<geometry_msgs::Transform>("output_transform",
			1);
	pub_transform_hololens=nh.advertise<geometry_msgs::PoseStamped>("/object",1);
	pub_fit_score = nh.advertise<std_msgs::Float32>("fit_score", 1);
	pub_do_tr = nh.advertise<std_msgs::Int32>("good_tr", 1);

	sub1 = new ros::Subscriber;
	sub2 = new ros::Subscriber;
	sub3 = new ros::Subscriber;

	listener1 = new tf::TransformListener;
	listener2 = new tf::TransformListener;
	listener3 = new tf::TransformListener;
	listener4 = new tf::TransformListener;

	listener1->waitForTransform("/cell_link", "camera1_rgb_optical_frame",
			ros::Time::now(), ros::Duration(1.0));
	listener2->waitForTransform("/cell_link", "camera2_rgb_optical_frame",
			ros::Time::now(), ros::Duration(1.0));
	listener3->waitForTransform("/cell_link", "camera3_rgb_optical_frame",
			ros::Time::now(), ros::Duration(1.0));
	listener4->waitForTransform("/world", "/cell_link", ros::Time::now(),
			ros::Duration(1.0));

	// AUTOMATIC TRANSFORM:
	listener1->lookupTransform("cell_link", "camera1_rgb_optical_frame",
			ros::Time::now(), transform1);
	listener2->lookupTransform("cell_link", "camera2_rgb_optical_frame",
			ros::Time::now(), transform2);
	listener3->lookupTransform("cell_link", "camera3_rgb_optical_frame",
			ros::Time::now(), transform3);
	listener4->lookupTransform("/world", "/cell_link", ros::Time::now(),
			transform4);

	if (pcl::io::loadOBJFile(
			"/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/car_pruebav2.obj",
			*initial_model) == -1) //* load the file
			{
		PCL_ERROR("Couldn't read file car_door.obj \n");
	}
	if (pcl::io::loadOBJFile(
			"/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/VITO_puerta_izq.obj",
			*initial_model_won) == -1) //* load the file
			{
		PCL_ERROR("Couldn't read file car_door.obj \n");
	}
	ROS_INFO_STREAM("Car door loaded!");

	// Initial transform to avoid local minima
	m = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ());
	//m=Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitZ());
	//m.setIdentity();
	trot.block<3, 3>(0, 0) = m;
	trot(0, 3) = 0;
	trot(1, 3) = 0;
	trot(2, 3) = 0;
	trot(3, 3) = 1;
	trot(3, 0) = 0;
	trot(3, 1) = 0;
	trot(3, 2) = 0;
	trasl << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.1, 0, 0, 0, 1;
	trtotal = trasl * trot;
	transf_total.setIdentity();
	//std::cout<<trtotal<<std::endl;
	pcl::transformPointCloudWithNormals(*initial_model, *model, trtotal);
	pcl::transformPointCloud(*initial_model_won, *model_won, trtotal);

	pcl::search::KdTree<PointType>::Ptr tree(
			new pcl::search::KdTree<PointType>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointType> ec;

	ros::Rate rate(10);
	while (ros::ok()) {

		if(cam_callbacks==true){

			if (cam_now == 1) {
				old_icp = ros::Time::now();
				*sub1 = nh.subscribe("/camera1/depth_registered/points", 1, cloud_cb1);
			} else if (cam_now == 2) {
				*sub2 = nh.subscribe("/camera2/depth_registered/points", 1, cloud_cb2);
			} else if (cam_now == 3) {
				*sub3 = nh.subscribe("/camera3/depth_registered/points", 1, cloud_cb3);
			}

		}else{

			ROS_WARN("T0");
			ROS_ERROR_STREAM((ros::Time::now()-old_icp).toSec());
			old_icp = ros::Time::now();

			//****************************************************

//			// Merge all three clouds
//			*cloud_merged  = *cloud1;
//			*cloud_merged += *cloud2;
//			*cloud_merged += *cloud3;

			//****************************************************

			// Convert the ROS sensor_msgs/PointCloud2 data to PCLPointCloud2
			pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2;
			pcl::toPCLPointCloud2(*cloud_merged, *pcl_cloud);

			// Perform actual downsampling filtering
			pcl::PCLPointCloud2ConstPtr pcl_cloud_ptr(pcl_cloud);
			pcl::PCLPointCloud2 pcl_cloud_downsampled;

			pcl::VoxelGrid<pcl::PCLPointCloud2> voxelgrid;
			voxelgrid.setInputCloud(pcl_cloud_ptr);
			// choose downsample, larger leaf size numbers, less points.
			// fully dependant on the actual cloud we are dealing with.
			// it should really be a config parameter.
			voxelgrid.setLeafSize(0.02, 0.02, 0.02);

			voxelgrid.filter(pcl_cloud_downsampled);

			pcl::fromPCLPointCloud2(pcl_cloud_downsampled, *cloud_filtered);

			//****************************************************

			//PASSTHROUGH FILTER
			pcl::PassThrough<PointType> pass;

//			pass.setInputCloud(cloud_merged);
//			pass.setFilterFieldName("x");
//			pass.setFilterLimits(-1.1, 1.1);
//			pass.filter(*cloud_filtered);
//
//			pass.setInputCloud(cloud_filtered);
//			pass.setFilterFieldName("y");
//			pass.setFilterLimits(-1.1, 1.1);
//			pass.filter(*cloud_filtered);

			pass.setInputCloud(cloud_filtered);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(0.01, 0.4);
			pass.filter(*cloud_filtered);

			// std::cout<<cloud_filtered->size()<<std::endl;

			//****************************************************

//			//FILTERING 1 OUT OF 2 POINTS IN THE PC:
//			pcl::ExtractIndices<PointType> extract2;
//			pcl::PointIndices::Ptr ind_down(new pcl::PointIndices());
//			c = 0;
//			for (int j = 0; j < (*cloud_filtered).size(); j++) {
//				if (c == 0) {
//					ind_down->indices.push_back(j);
//					c = 1;
//				} else
//					c = 0;
//			}
//			extract2.setInputCloud(cloud_filtered);
//			extract2.setIndices(ind_down);
//			extract2.setNegative(false);
//			extract2.filter(*cloud_filtered);

			//****************************************************

			//PERFORMING ACTUAL ICP ALGORITHM:

			//auto start = std::chrono::high_resolution_clock::now();
			pcl::IterativeClosestPoint<PointType, PointType> icp;
			icp.setInputSource(model_won);
			icp.setInputTarget(cloud_filtered);
			//icp.setTransformationEpsilon(0);
			icp.setEuclideanFitnessEpsilon(1e-5);
			icp.setMaximumIterations(150);

			ROS_WARN("T1");
			ROS_ERROR_STREAM((ros::Time::now()-old_icp).toSec());
			old_icp = ros::Time::now();

			pcl::PointCloud<PointType> Final;
			icp.align(Final);

			ROS_WARN("T2");
			ROS_ERROR_STREAM((ros::Time::now()-old_icp).toSec());
			old_icp = ros::Time::now();

		    std::cout << " score: " << icp.getFitnessScore() << std::endl;
			//  std::cout<<icp.euclidean_fitness_epsilon_<<std::endl;
			std::cout << icp.nr_iterations_ << std::endl;
			Eigen::Matrix4f transf;
			transf = icp.getFinalTransformation();
			fitness_score.data = icp.getFitnessScore();
			pcl::transformPointCloudWithNormals(*model, *adjusted_model,
					transf);
			pcl::transformPointCloud(*model_won, *adjusted_model_won, transf);
			fit = icp.getFitnessScore();
			iterac = icp.nr_iterations_;
//			if (iterac > 15) {
//				prev_fit = 1;
//			}
//			if ((prev_fit - fit) > 5e-6 && fit < 3.0e-4) {
//				do_tr.data = 1;
			if ( fit < 3.0e-4) {
				do_tr.data = 1;
			} else {
				do_tr.data = 0;
			}

			ROS_WARN("T3");
			ROS_ERROR_STREAM((ros::Time::now()-old_icp).toSec());
			old_icp = ros::Time::now();

			/*std::cout<<(*adjusted_model).points[5].x<<std::endl;
			 std::cout<<(*adjusted_model).points[5].y<<std::endl;
			 std::cout<<(*adjusted_model).points[5].z<<std::endl;
			 std::cout<<(*adjusted_model).points[5].normal_x<<std::endl;
			 std::cout<<(*adjusted_model).points[5].normal_y<<std::endl;
			 std::cout<<(*adjusted_model).points[5].normal_z<<std::endl;*/

			pcl_ros::transformAsMatrix(transform4, eigen_transform4);
			transf_total = transf * transf_total;
			tr_msg = eigen_transform4 * transf_total * trtotal;
			Eigen::Quaternionf qm(tr_msg.block<3, 3>(0, 0));
			Eigen::Vector3f vm(tr_msg.block<3, 1>(0, 3));
			tf::Quaternion qm2(qm.x(), qm.y(), qm.z(), qm.w());
			tf::Vector3 vm2(vm.x(), vm.y(), vm.z());
			tf_model.rotation.x = qm2.x();
			tf_model.rotation.y = qm2.y();
			tf_model.rotation.z = qm2.z();
			tf_model.rotation.w = qm2.w();
			tf_model.translation.x = vm2.x();
			tf_model.translation.y = vm2.y();
			tf_model.translation.z = vm2.z();
			tf_hololens.pose.position.x= vm2.x();
			tf_hololens.pose.position.y= vm2.y();
			tf_hololens.pose.position.z= vm2.z();
			tf_hololens.pose.orientation.x=qm2.x();
			tf_hololens.pose.orientation.y=qm2.y();
			tf_hololens.pose.orientation.z=qm2.z();
			tf_hololens.pose.orientation.w=qm2.w();

			//std::cout<<qm2.x()<<" "<<qm2.y()<<" "<<qm2.z()<<" "<<qm2.w()<<std::endl;
			//std::cout<<vm2.x()<<" "<<vm2.y()<<" "<<vm2.z()<<std::endl;
			seq++;

			pcl::toROSMsg(*adjusted_model, output1);
			//output1.header = input1.header;
			//output1.header = cloud_in1.header;
			output1.header.frame_id = "/cell_link";
			output1.header.stamp = ros::Time::now();
			output1.header.seq = seq;

			pcl::toROSMsg(*cloud_filtered, output_merged);
			//output_merged.header = input1.header;
			//output_merged.header = cloud_in1.header;
			output_merged.header.frame_id = "/cell_link";
			output_merged.header.stamp = ros::Time::now();
			output_merged.header.seq = seq;

			pub.publish(output1);
			pub_m.publish(output_merged);
			pub_transform.publish(tf_model);
			pub_transform_hololens.publish(tf_hololens);
			pub_fit_score.publish(fitness_score);
			pub_do_tr.publish(do_tr);


			*model = *adjusted_model;
			*model_won = *adjusted_model_won;
			//prev_it = iterac;
			//prev_fit = fit;

			cam_callbacks = true;

			ROS_WARN("T4");
			ROS_ERROR_STREAM((ros::Time::now()-old_icp).toSec());
		}

		// Spin
		rate.sleep();
		ros::spinOnce();
	}

	// Free the allocated memory
	delete sub1;
	delete sub2;
	delete sub3;
	delete listener1;
	delete listener2;
	delete listener3;
	delete listener4;
}

