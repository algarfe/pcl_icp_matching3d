/*
 * multiPC_ICP.cpp
 *
 *  Created on: Mar 6, 2019
 *      Author: idf
 */
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl/keypoints/iss_3d.h>
#include <pthread.h>
#include <pcl/registration/icp.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <chrono>
#include <math.h>

typedef pcl::Normal NormalType;
typedef pcl::PointXYZ PointType;


pcl::PointCloud<PointType>::Ptr cloud1 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud2 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud3 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud4 (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_merged (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr adjusted_model (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr initial_model (new pcl::PointCloud<PointType> ());



Eigen::Matrix4f trasl, trtotal, trasl2,trtotal2;
Eigen::MatrixXf trot(4,4), trot2(4,4);
Eigen::Matrix3f m,m2;

sensor_msgs::PointCloud2 input1;
sensor_msgs::PointCloud2 cloud_in;

sensor_msgs::PointCloud2 output1;

sensor_msgs::PointCloud2 output_merged;

ros::Publisher pub;
ros::Publisher pub_m;

int k=0;
int c=0;

bool cloud_new=false;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{
	cloud_in = *cloud_msg1;
	cloud_new = true;
}


int
main (int argc, char** argv)
{
	// Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


  pub = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
  pub_m = nh.advertise<sensor_msgs::PointCloud2> ("output_merged", 1);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  listener.waitForTransform("world","camera1_rgb_optical_frame",ros::Time::now(),ros::Duration(1.0));

  ros::Subscriber sub = nh.subscribe  ("/camera1/depth_registered/points",1,cloud_cb);

  pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;

  if (pcl::io::loadOBJFile ("/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/car_door.obj", *initial_model) == -1) //* load the file
  {
	 PCL_ERROR ("Couldn't read file car_door.obj \n");
  }
  ROS_INFO_STREAM("Puerta cargada");

 //   m=Eigen::AngleAxisf(-M_PI/2,Eigen::Vector3f::UnitY())*Eigen::AngleAxisf(-M_PI/2,Eigen::Vector3f::UnitZ());
	m=Eigen::AngleAxisf(M_PI_2,Eigen::Vector3f::UnitX());
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
   // std::cout<<trtotal<<std::endl;
   pcl::transformPointCloud (*initial_model, *model, trtotal);


  ros::Rate rate(100);

  while(ros::ok())
  {

	  if(cloud_new==true)
	  {

		// Automatic transform
		listener.lookupTransform("world","camera1_rgb_optical_frame", ros::Time::now(), transform);
		Eigen::Matrix4f eigen_transform;
		pcl_ros::transformAsMatrix (transform, eigen_transform);
		std::cout<<eigen_transform<<std::endl;
		pcl_ros::transformPointCloud(eigen_transform,cloud_in,input1);
		input1.header.frame_id="/world";


		pcl::fromROSMsg(input1,  *cloud1);

		*cloud_merged=*cloud1;

		//PASSTHROUGH FILTER
		pcl::PassThrough<PointType> pass;

	 	pass.setInputCloud (cloud_merged);
	 	pass.setFilterFieldName ("x");
	 	pass.setFilterLimits (-0.9, 0.9);
	 	pass.filter (*cloud_filtered);

	 	pass.setInputCloud (cloud_filtered);
	 	pass.setFilterFieldName ("y");
	 	pass.setFilterLimits (-0.96, 0.96);
	 	pass.filter (*cloud_filtered);

	 	pass.setInputCloud (cloud_filtered);
	 	pass.setFilterFieldName ("z");
	 	pass.setFilterLimits (0.65, 1);
	 	pass.filter (*cloud_filtered);

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

		//CLUSTER EXTRACTION:
		tree->setInputCloud (cloud_filtered);
		pcl::ExtractIndices<PointType> extract;

		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (100);
		ec.setMaxClusterSize (900000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud_filtered);
		ec.extract (cluster_indices);
		//	 std::cout<<cluster_indices.size()<<std::endl;
		//Loading cluster indices
		pcl::IndicesPtr indices_ptr (new std::vector<int> (cluster_indices[0].indices.size ()));
		for (int j = 0; j < indices_ptr->size(); j++)
		(*indices_ptr)[j] = cluster_indices[0].indices[j];

		//Removing points which don't belong to the main cluster:
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (indices_ptr);
		extract.setNegative (false);
		extract.filter (*cloud_filtered);

		auto start = std::chrono::high_resolution_clock::now();
		pcl::IterativeClosestPoint<PointType, PointType> icp;
		icp.setInputSource(model);
		icp.setInputTarget(cloud_filtered);
		//icp.setTransformationEpsilon(0);
		icp.setEuclideanFitnessEpsilon(1e-5);
		icp.setMaximumIterations(150);
		pcl::PointCloud<PointType> Final;
		icp.align(Final);
		//  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		// icp.getFitnessScore() << std::endl;
		//  std::cout<<icp.euclidean_fitness_epsilon_<<std::endl;

		//  std::cout<<icp.nr_iterations_<<std::endl;
		Eigen::Matrix4f transf;
		transf=icp.getFinalTransformation();

		pcl::transformPointCloud (*model, *adjusted_model, transf);

		pcl::toROSMsg(*adjusted_model,output1);
		output1.header=input1.header;
		pcl::toROSMsg(*cloud_filtered,output_merged);
		output_merged.header=input1.header;

		pub.publish (output1);
		pub_m.publish(output_merged);

		cloud_new=false;
	  }

		rate.sleep();
		ros::spinOnce();
  }


  // Spin
  //ros::spin ();
}



