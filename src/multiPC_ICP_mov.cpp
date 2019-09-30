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
#include <gazebo_msgs/ModelStates.h>


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
gazebo_msgs::ModelStates info_door;


sensor_msgs::PointCloud2 output1;

sensor_msgs::PointCloud2 output_merged;

ros::Publisher pub;
ros::Publisher pub_m;

int k=0;
int c=0;
int p;
float twist[2];

void subscriber_cb(gazebo_msgs::ModelStates msg){
	for(int j=0; j<4;j++){
		if(msg.name[j]=="car_door_base"){
			p=j;
		}
	}
	twist[0]=msg.twist[p].linear.x;
	twist[1]=msg.twist[p].linear.y;
}
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg1,const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
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

	  pcl::fromROSMsg(input1,  *cloud1);
	  pcl::fromROSMsg(input2, *cloud2);

	  *cloud_merged=*cloud1;
	  *cloud_merged += *cloud2;

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
		  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		  icp.getFitnessScore() << std::endl;
		//  std::cout<<icp.euclidean_fitness_epsilon_<<std::endl;

		//  std::cout<<icp.nr_iterations_<<std::endl;
		  Eigen::Matrix4f transf;
		  transf=icp.getFinalTransformation();
		  auto finish = std::chrono::high_resolution_clock::now();
		  std::chrono::duration<double> elapsed = finish - start;
		  transf(0,3)=transf(0,3)+twist[0]*float(elapsed.count());
		  transf(1,3)=transf(1,3)+twist[1]*float(elapsed.count());

		  pcl::transformPointCloud (*model, *adjusted_model, transf);

		    pcl::toROSMsg(*adjusted_model,output1);
		 	  output1.header=input1.header;
	  pcl::toROSMsg(*cloud_filtered,output_merged);
	  output_merged.header=input1.header;


		pub.publish (output1);

		pub_m.publish(output_merged);

	/*	if(icp.getFitnessScore()<1e-4){
		*model=*adjusted_model;}*/


}


int
main (int argc, char** argv)
{
	// Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


  pub = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
  pub_m = nh.advertise<sensor_msgs::PointCloud2> ("output_merged", 1);

  ros::Subscriber sub = nh.subscribe ("/gazebo/model_states", 1, subscriber_cb);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc1 (nh,"/camera2/depth/points",1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc2 (nh,"/camera4/depth/points",1);


  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync(pc1, pc2, 10);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  // Spin
  ros::spin ();
}



