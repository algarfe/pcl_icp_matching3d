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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
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


pcl::PointCloud<PointType>::Ptr cloud1 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud2 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud3 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_merged (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType> ());
pcl::PointCloud<NormalType>::Ptr model (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr adjusted_model (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr initial_model (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<PointType>::Ptr model_won (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr adjusted_model_won (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr initial_model_won (new pcl::PointCloud<PointType> ());


Eigen::Matrix4f trasl, trtotal, transf_total;
Eigen::MatrixXf trot(4,4), tr_msg(4,4);
Eigen::Matrix3f m;

sensor_msgs::PointCloud2 input1;
sensor_msgs::PointCloud2 input2;
sensor_msgs::PointCloud2 input3;
sensor_msgs::PointCloud2 cloud_in1;
sensor_msgs::PointCloud2 cloud_in2;
sensor_msgs::PointCloud2 cloud_in3;
Eigen::Matrix4f eigen_transform, eigen_transform2, eigen_transform3;
tf::StampedTransform transform;
tf::StampedTransform transform2;
tf::StampedTransform transform3;

sensor_msgs::PointCloud2 output1;

sensor_msgs::PointCloud2 output_merged;
geometry_msgs::Transform tf_model;

std_msgs::Float32 fitness_score;
std_msgs::Int32 do_tr;

ros::Publisher pub;
ros::Publisher pub_m;
ros::Publisher pub_transform;
ros::Publisher pub_fit_score;
ros::Publisher pub_do_tr;

int k=0;
int c=0;
float prev_fit=1, fit;
int prev_it, iterac;

bool cloud_new=false;
bool cloud_new2=false;
bool cloud_new3=false;

bool hard_code=false;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{
	cloud_in1=*cloud_msg1;
	cloud_new = true;

}
void
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
	cloud_in2=*cloud_msg2;
	cloud_new2 = true;

}
void
cloud_cb3 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg3)
{
	cloud_in3=*cloud_msg3;
	cloud_new3 = true;

}

void
hard_coded(){
	//Hard-coded solution for a particular positioning of the kinect cameras
	//tf::Quaternion q(-0.0251629,0.960932,-0.275627,-0.00240536);
	//tf::Vector3 v(-0.00494249,0.922884,1.98083);
	//tf::Quaternion q2(0.958674,0.0167337,-0.00495675,-0.283972);
	//tf::Vector3 v2(0.00497259,-0.92343,1.96);
	tf::Quaternion q(0.00833402,0.954984,-0.29653,-0.00258778);
	tf::Vector3 v(-0.00499315,0.960785,1.98);
	tf::Quaternion q2(0.953572,0.0166447,-0.00524804,-0.30066);
	tf::Vector3 v2(0.00497259,-0.93843,1.94);
	transform.setRotation(q);
	transform.setOrigin(v);
	transform2.setRotation(q2);
	transform2.setOrigin(v2);
}


int
main (int argc, char** argv)
{
	// Initialize ROS
  ros::init (argc, argv, "multiPC_ICP");
  ros::NodeHandle nh;


  pub = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
  pub_m = nh.advertise<sensor_msgs::PointCloud2> ("output_merged_2", 1);
  pub_transform = nh.advertise<geometry_msgs::Transform> ("output_transform", 1);
  pub_fit_score=nh.advertise<std_msgs::Float32> ("fit_score",1);
  pub_do_tr=nh.advertise<std_msgs::Int32> ("good_tr",1);

  tf::TransformListener listener;
  tf::TransformListener listener2;
  tf::TransformListener listener3;

  listener.waitForTransform("/world","camera1_rgb_optical_frame",ros::Time::now(),ros::Duration(1.0));
  listener2.waitForTransform("/world","camera2_rgb_optical_frame",ros::Time::now(),ros::Duration(1.0));
  listener3.waitForTransform("/world","camera3_rgb_optical_frame",ros::Time::now(),ros::Duration(1.0));

  if (pcl::io::loadOBJFile ("/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/car_pruebav2.obj", *initial_model) == -1) //* load the file
  {
	PCL_ERROR ("Couldn't read file car_door.obj \n");
  }
  if (pcl::io::loadOBJFile ("/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/VITO_puerta_izq.obj", *initial_model_won) == -1) //* load the file
  {
	PCL_ERROR ("Couldn't read file car_door.obj \n");
  }
  ROS_INFO_STREAM("Puerta cargada");
  //m=Eigen::AngleAxisf(-M_PI/2,Eigen::Vector3f::UnitZ());
  //m=Eigen::AngleAxisf(M_PI_2,Eigen::Vector3f::UnitX());
  m.setIdentity();
  trot.block<3,3>(0,0)=m;
  trot(0,3)=0;
  trot(1,3)=0;
  trot(2,3)=0;
  trot(3,3)=1;
  trot(3,0)=0;
  trot(3,1)=0;
  trot(3,2)=0;
  trasl<<1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0.8,
		0, 0, 0, 1;
  trtotal=trasl*trot;
  transf_total.setIdentity();
  //std::cout<<trtotal<<std::endl;
  pcl::transformPointCloudWithNormals (*initial_model, *model, trtotal);
  pcl::transformPointCloud(*initial_model_won,*model_won,trtotal);

  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;


  ros::Subscriber sub = nh.subscribe("/camera1/depth_registered/points",1,cloud_cb);
  ros::Subscriber sub2 = nh.subscribe ("/camera2/depth_registered/points",1,cloud_cb2);
  ros::Subscriber sub3 = nh.subscribe ("/camera3/depth_registered/points",1,cloud_cb3);


  ros::Rate rate(100);
  while(ros::ok())
  {
	  if(cloud_new==true && cloud_new2==true && cloud_new3==true)
	  	  {
  // AUTOMATIC TRANSFORM:
  if(hard_code==true){
	  hard_coded();
  }
  else{
  listener.lookupTransform("world","camera1_rgb_optical_frame", ros::Time::now(), transform);
  listener2.lookupTransform("world","camera2_rgb_optical_frame", ros::Time::now(), transform2);
  listener3.lookupTransform("world","camera3_rgb_optical_frame", ros::Time::now(), transform3);
  }

  pcl_ros::transformAsMatrix (transform, eigen_transform);
//  std::cout<<eigen_transform<<std::endl;
  pcl_ros::transformPointCloud(eigen_transform,cloud_in1,input1);
  input1.header.frame_id="/world";
  pcl_ros::transformAsMatrix (transform2, eigen_transform2);
  //std::cout<<eigen_transform2<<std::endl;
  pcl_ros::transformPointCloud(eigen_transform2,cloud_in2,input2);
  pcl_ros::transformAsMatrix (transform3, eigen_transform3);
  pcl_ros::transformPointCloud(eigen_transform3,cloud_in3,input3);
  input3.header.frame_id="/world";

  pcl::fromROSMsg(input1,  *cloud1);
  pcl::fromROSMsg(input2, *cloud2);
  pcl::fromROSMsg(input3, *cloud3);



  pcl::PassThrough<PointType> pass_cloud1;
  pass_cloud1.setInputCloud (cloud1);
  pass_cloud1.setFilterFieldName ("y");
  pass_cloud1.setFilterLimits (-0.1, 0.96);
  pass_cloud1.filter (*cloud1);
  pass_cloud1.setInputCloud (cloud1);
  pass_cloud1.setFilterFieldName ("x");
  pass_cloud1.setFilterLimits (-1, 0.1);
  pass_cloud1.filter (*cloud1);

  pcl::PassThrough<PointType> pass_cloud2;
  pass_cloud2.setInputCloud (cloud2);
  pass_cloud2.setFilterFieldName ("y");
  pass_cloud2.setFilterLimits (-0.96, 0.1);
  pass_cloud2.filter (*cloud2);
  pass_cloud2.setInputCloud (cloud2);
  pass_cloud2.setFilterFieldName ("x");
  pass_cloud2.setFilterLimits (-1, 0.1);
  pass_cloud2.filter (*cloud2);

  pcl::PassThrough<PointType> pass_cloud3;
  pass_cloud3.setInputCloud (cloud3);
  pass_cloud3.setFilterFieldName ("x");
  pass_cloud3.setFilterLimits (-0.1, 0.9);
  pass_cloud3.filter (*cloud3);
//  pass_cloud3.setInputCloud (cloud3);
//  pass_cloud3.setFilterFieldName ("z");
//  pass_cloud3.setFilterLimits (0.68,1);
//  pass_cloud3.filter (*cloud3);

  *cloud_merged=*cloud1;
  *cloud_merged += *cloud2;
  *cloud_merged += *cloud3;

  //PASSTHROUGH FILTER
  pcl::PassThrough<PointType> pass;

  pass.setInputCloud (cloud_merged);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.2, 1.2);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.96, 0.96);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.64, 1);
  pass.filter (*cloud_filtered);

 // std::cout<<cloud_filtered->size()<<std::endl;

  //FILTERING 1 OUT OF 2 POINTS IN THE PC:
  pcl::ExtractIndices<PointType> extract2;
  pcl::PointIndices::Ptr ind_down(new pcl::PointIndices());
  c=0;
  for (int j = 0; j < (*cloud_filtered).size(); j++)
  {
  	 if (c==0){
 	 ind_down->indices.push_back(j);
 	 c=1;
    	 }
   	 else
 	 c=0;
         }
  extract2.setInputCloud (cloud_filtered);
  extract2.setIndices (ind_down);
  extract2.setNegative (false);
  extract2.filter (*cloud_filtered);

  //PERFORMING ACTUAL ICP ALGORITHM:

  auto start = std::chrono::high_resolution_clock::now();
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(model_won);
  icp.setInputTarget(cloud_filtered);
  //icp.setTransformationEpsilon(0);
  icp.setEuclideanFitnessEpsilon(1e-5);
  icp.setMaximumIterations(150);
  pcl::PointCloud<PointType> Final;
  icp.align(Final);
  //  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  // icp.getFitnessScore() << std::endl;
  //  std::cout<<icp.euclidean_fitness_epsilon_<<std::endl;
   std::cout<<icp.nr_iterations_<<std::endl;
  Eigen::Matrix4f transf;
  transf=icp.getFinalTransformation();
  fitness_score.data=icp.getFitnessScore();
  pcl::transformPointCloudWithNormals (*model, *adjusted_model, transf);
  pcl::transformPointCloud (*model_won, *adjusted_model_won, transf);
  fit=icp.getFitnessScore();
  iterac=icp.nr_iterations_;
  if(iterac>50){
	  prev_fit=1;
  }
  if(fit<prev_fit && fit<3.5e-4){
	  do_tr.data=1;
  }
  else{
	  do_tr.data=0;
  }

  /*std::cout<<(*adjusted_model).points[5].x<<std::endl;
  std::cout<<(*adjusted_model).points[5].y<<std::endl;
  std::cout<<(*adjusted_model).points[5].z<<std::endl;
  std::cout<<(*adjusted_model).points[5].normal_x<<std::endl;
  std::cout<<(*adjusted_model).points[5].normal_y<<std::endl;
  std::cout<<(*adjusted_model).points[5].normal_z<<std::endl;*/

  transf_total=transf*transf_total;
  tr_msg=transf_total*trtotal;
  Eigen::Quaternionf qm(tr_msg.block<3,3>(0,0));
  Eigen::Vector3f vm(tr_msg.block<3,1>(0,3));
  tf::Quaternion qm2(qm.x(),qm.y(),qm.z(),qm.w());
  tf::Vector3 vm2(vm.x(),vm.y(),vm.z());
  tf_model.rotation.x=qm2.x();
  tf_model.rotation.y=qm2.y();
  tf_model.rotation.z=qm2.z();
  tf_model.rotation.w=qm2.w();
  tf_model.translation.x=vm2.x();
  tf_model.translation.y=vm2.y();
  tf_model.translation.z=vm2.z();

  //std::cout<<qm2.x()<<" "<<qm2.y()<<" "<<qm2.z()<<" "<<qm2.w()<<std::endl;
  //std::cout<<vm2.x()<<" "<<vm2.y()<<" "<<vm2.z()<<std::endl;
  pcl::toROSMsg(*adjusted_model,output1);
  output1.header=input1.header;
  pcl::toROSMsg(*cloud_filtered,output_merged);
  output_merged.header=input1.header;

  pub.publish (output1);
  pub_m.publish(output_merged);
  pub_transform.publish(tf_model);
  pub_fit_score.publish(fitness_score);
  pub_do_tr.publish(do_tr);
  cloud_new=false;
  cloud_new2=false;
  *model=*adjusted_model;
  *model_won=*adjusted_model_won;
  prev_it=iterac;
  prev_fit=fit;
	  	  }
  // Spin
  rate.sleep();
  ros::spinOnce();
  }
}



