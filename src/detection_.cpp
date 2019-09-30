/*
 * detection.cpp
 *
 *  Created on: Feb 6, 2019
 *      Author: idf
 */
// PCL specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <iostream>

//typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

#define model_sampl_size 0.01
#define cloud_sampl_size 0.01
#define descr_rad 0.02
#define use_hough true
#define rf_rad 0.015
#define cg_size_ 0.1f
#define cg_thresh_ 1.0f
#define show_correspondences_ true
#define show_keypoints_ true


pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_keypoints (new pcl::PointCloud<PointType> ());
pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr cloud_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
pcl::PointCloud<DescriptorType>::Ptr cloud_descriptors (new pcl::PointCloud<DescriptorType> ());
pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
int k=0;
ros::Publisher pub;

void loading_model (){
	//loading the model Point Cloud



}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{	       sensor_msgs::PointCloud2 output;


	if (k==0){
		  if (pcl::io::loadPCDFile ("/home/idf/ros/catkin_ws/prueba_captura.pcd", *model) == -1) //* load the file
			    	  {
			    	    PCL_ERROR ("Couldn't read file prueba_captura.pcd \n");
			    	  }
			    	  ROS_INFO_STREAM("Imagen cargada");
			    	  pcl::toROSMsg(*model,output);
			    	  output.header=cloud_msg->header;
			    	  k++;
	}
	/*

		      	       pub.publish (output);
*/
	else{
	  //data container for the Point Cloud
	   pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType> ());
	  //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	  // Convert to PCL data type
	  pcl::fromROSMsg(*cloud_msg, *cloud);
	  std::cout<<cloud->size()<<std::endl;
	  //Normal Estimation:
	  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	  norm_est.setKSearch (10);
	  norm_est.setInputCloud (model);
	  norm_est.compute (*model_normals);
	  norm_est.setInputCloud (cloud);
	  norm_est.compute (*cloud_normals);


	  //Downsampling model and input cloud
	  pcl::UniformSampling<PointType> uniform_sampling;

	    uniform_sampling.setInputCloud (model);
	    uniform_sampling.setRadiusSearch (model_sampl_size);
	  //  uniform_sampling.filter (*model_keypoints);
	    pcl::PointCloud<int> keypointIndices1;
	    uniform_sampling.compute(keypointIndices1);
	    pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);

	    uniform_sampling.setInputCloud (cloud);
	    uniform_sampling.setRadiusSearch (cloud_sampl_size);
	//    uniform_sampling.filter (*cloud_keypoints);
	    pcl::PointCloud<int> keypointIndices2;
	    uniform_sampling.compute(keypointIndices2);
	    pcl::copyPointCloud(*cloud, keypointIndices2.points, *cloud_keypoints);

	    std::cout<<"Modelos sampleados"<<std::endl;

	    //Compute Descriptor for keypoints
	    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	    descr_est.setRadiusSearch (descr_rad);

	    descr_est.setInputCloud (model_keypoints);
	    descr_est.setInputNormals (model_normals);
	    descr_est.setSearchSurface (model);
	    descr_est.compute (*model_descriptors);

	    descr_est.setInputCloud (cloud_keypoints);
	    descr_est.setInputNormals (cloud_normals);
	    descr_est.setSearchSurface (cloud);
	    descr_est.compute (*cloud_descriptors);

	    //Find Model-Scene Correspondences with KdTree
	    std::cout<<"Descriptores keypoints computados"<<std::endl;
	    pcl::CorrespondencesPtr model_cloud_corrs (new pcl::Correspondences ());

	      pcl::KdTreeFLANN<DescriptorType> match_search;
	      match_search.setInputCloud (model_descriptors);

	      //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	      for (size_t i = 0; i < cloud_descriptors->size (); ++i)
	      {
	        std::vector<int> neigh_indices (1);
	        std::vector<float> neigh_sqr_dists (1);
	        if (!std::isfinite (cloud_descriptors->at (i).descriptor[0])) //skipping NaNs
	        {
	          continue;
	        }
	        int found_neighs = match_search.nearestKSearch (cloud_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
	        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
	        {
	          pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
	          model_cloud_corrs->push_back (corr);
	        }
	      }
	      std::cout<<"Correspondencias encontradas"<<std::endl;
	      //
	       //  Actual Clustering
	       //
	       std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	       std::vector<pcl::Correspondences> clustered_corrs;

	       //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations
	       //std::vector<pcl::Correspondences> &clustered_corrs
	       //  Using Hough3D
	       if (use_hough)
	       {
	         //
	         //  Compute (Keypoints) Reference Frames only for Hough
	         //
	         pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
	         pcl::PointCloud<RFType>::Ptr cloud_rf (new pcl::PointCloud<RFType> ());

	         pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
	         rf_est.setFindHoles (true);
	         rf_est.setRadiusSearch (rf_rad);

	         rf_est.setInputCloud (model_keypoints);
	         rf_est.setInputNormals (model_normals);
	         rf_est.setSearchSurface (model);
	         rf_est.compute (*model_rf);

	         rf_est.setInputCloud (cloud_keypoints);
	         rf_est.setInputNormals (cloud_normals);
	         rf_est.setSearchSurface (cloud);
	         rf_est.compute (*cloud_rf);

	         //  Clustering
	         pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
	         clusterer.setHoughBinSize (cg_size_);
	         clusterer.setHoughThreshold (cg_thresh_);
	         clusterer.setUseInterpolation (true);
	         clusterer.setUseDistanceWeight (false);

	         clusterer.setInputCloud (model_keypoints);
	         clusterer.setInputRf (model_rf);
	         clusterer.setSceneCloud (cloud_keypoints);
	         clusterer.setSceneRf (cloud_rf);
	         clusterer.setModelSceneCorrespondences (model_cloud_corrs);

	         clusterer.recognize (rototranslations, clustered_corrs);
	         std::cout<<"Obtenidas rototranslations"<<std::endl;
	       }
	       else // Using GeometricConsistency
	       {
	         pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
	         gc_clusterer.setGCSize (cg_size_);
	         gc_clusterer.setGCThreshold (cg_thresh_);

	         gc_clusterer.setInputCloud (model_keypoints);
	         gc_clusterer.setSceneCloud (cloud_keypoints);
	         gc_clusterer.setModelSceneCorrespondences (model_cloud_corrs);

	         gc_clusterer.recognize (rototranslations, clustered_corrs);
	       }

		      std::cout<<rototranslations.size()<<std::endl;

	       for (size_t i = 0; i < rototranslations.size (); ++i)
	       {

	         pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
	      //   std::cout<<i<<std::endl;

	       }


	       // Convert to ROS data type
	      	       pcl::toROSMsg(*rotated_model,output);
	      	    //   output.header=cloud_msg->header;
}
  pub.publish (output);

}

int
main (int argc, char** argv)
{


	// Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

