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


sensor_msgs::PointCloud2 input1;
sensor_msgs::PointCloud2 input2;
sensor_msgs::PointCloud2 input3;
sensor_msgs::PointCloud2 input4;


sensor_msgs::PointCloud2 output1;

sensor_msgs::PointCloud2 output_merged;

ros::Publisher pub;
ros::Publisher pub_m;

int k=0;
int c=0;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg1,const sensor_msgs::PointCloud2ConstPtr& cloud_msg2, const sensor_msgs::PointCloud2ConstPtr& cloud_msg3,const sensor_msgs::PointCloud2ConstPtr& cloud_msg4)
{

if(k==0){
	  if (pcl::io::loadOBJFile ("/home/idf/ros/catkin_ws/car_door.obj", *model) == -1) //* load the file
		    	  {
		    	    PCL_ERROR ("Couldn't read file car_door.obj \n");
		    	  }
	 		    	  ROS_INFO_STREAM("Puerta cargada");
		    	  k=1;
}

	tf::TransformListener listener;

	listener.waitForTransform("/world",cloud_msg1->header.frame_id,cloud_msg1->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud("/world",*cloud_msg1,input1,listener);

	listener.waitForTransform("/world",cloud_msg2->header.frame_id,cloud_msg2->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud("/world",*cloud_msg2,input2,listener);

	listener.waitForTransform("/world",cloud_msg3->header.frame_id,cloud_msg3->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud("/world",*cloud_msg3,input3,listener);

	listener.waitForTransform("/world",cloud_msg4->header.frame_id,cloud_msg4->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud("/world",*cloud_msg4,input4,listener);

	  pcl::fromROSMsg(input1,  *cloud1);
	  pcl::fromROSMsg(input2, *cloud2);
	  pcl::fromROSMsg(input3, *cloud3);
	  pcl::fromROSMsg(input4, *cloud4);

	  *cloud_merged=*cloud1;
	  *cloud_merged += *cloud2;
	  *cloud_merged += *cloud3;
	  *cloud_merged += *cloud4;

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




}


int
main (int argc, char** argv)
{
	// Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


  pub = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
  pub_m = nh.advertise<sensor_msgs::PointCloud2> ("output_merged", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc1 (nh,"/camera1/depth/points",1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc2 (nh,"/camera2/depth/points",1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc3 (nh,"/camera3/depth/points",1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc4 (nh,"/camera4/depth/points",1);


  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> sync(pc1, pc2, pc3, pc4, 10);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2, _3, _4));

  // Spin
  ros::spin ();
}



