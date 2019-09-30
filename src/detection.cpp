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
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
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
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pthread.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>





typedef pcl::Normal NormalType;
typedef pcl::PointXYZ PointType;
typedef pcl::FPFHSignature33 DescriptorType;

pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr model_filtered (new pcl::PointCloud<PointType> ());
pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr cloud_normals (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
pcl::PointCloud<DescriptorType>::Ptr cloud_descriptors (new pcl::PointCloud<DescriptorType> ());
pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
pcl::PointCloud<PointType>::Ptr cloud_keypoints (new pcl::PointCloud<PointType> ());
std::vector<pcl::PointIndices> cluster_indices_model;
pcl::ISSKeypoint3D<PointType, PointType> iss_detector_model;
pcl::FPFHEstimation<PointType, NormalType, DescriptorType> fpfh_model;
pcl::PointCloud<DescriptorType>::Ptr  model_descriptors2 (new pcl::PointCloud<DescriptorType>  ()) ;
pcl::PointCloud<DescriptorType>::Ptr cloud_descriptors2 (new pcl::PointCloud<DescriptorType> ()) ;
boost::shared_ptr<std::vector<int> > cloud_keypoints_pers (new std::vector<int> ());
boost::shared_ptr<std::vector<int> > model_keypoints_pers (new std::vector<int> ());


std::vector< int >  ind;
int k=0, c=0;
ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;


//Obtains PC resolution (from 'pcl:correspondence_grouping.cpp')
 double
 computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
 {
   double res = 0.0;
   int n_points = 0;
   int nres;
   std::vector<int> indices (2);
   std::vector<float> sqr_distances (2);
   pcl::search::KdTree<PointType> tree;
   tree.setInputCloud (cloud);

   for (size_t i = 0; i < cloud->size (); ++i)
   {
     if (! pcl_isfinite ((*cloud)[i].x))
     {
       continue;
     }
     //Considering the second neighbor since the first is the point itself.
     nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
     if (nres == 2)
     {
       res += sqrt (sqr_distances[1]);
       ++n_points;
     }
   }
   if (n_points != 0)
   {
     res /= n_points;
   }
   return res;
 }



void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
		sensor_msgs::PointCloud2 output;
		sensor_msgs::PointCloud2 output2;
		sensor_msgs::PointCloud2 output3;

		pcl::PassThrough<PointType> pass;
	 	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	 	std::vector<pcl::PointIndices> cluster_indices;
	 	pcl::EuclideanClusterExtraction<PointType> ec;
	 	pcl::ExtractIndices<PointType> extract;
	 	pcl::ExtractIndices<PointType> extract2;
	 	pcl::search::KdTree<PointType>::Ptr tree_kp (new pcl::search::KdTree<PointType> ());
	 	pcl::ISSKeypoint3D<PointType, PointType> iss_detector;
		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
        pcl::FPFHEstimation<PointType, NormalType, DescriptorType> fpfh;
	 	pcl::search::KdTree<PointType>::Ptr tree_fpfh (new pcl::search::KdTree<PointType> ());
	 	pcl::search::KdTree<PointType>::Ptr tree_rejnorm (new pcl::search::KdTree<PointType> ());
	 	pcl::MultiscaleFeaturePersistence<PointType, DescriptorType> fper_cloud;
	 	pcl::MultiscaleFeaturePersistence<PointType, DescriptorType> fper;
	 	pcl::FPFHEstimation<PointType, NormalType, DescriptorType>::Ptr fest_model (new pcl::FPFHEstimation<PointType, NormalType, DescriptorType>);
	 	pcl::FPFHEstimation<PointType, NormalType, DescriptorType>::Ptr fest_cloud (new pcl::FPFHEstimation<PointType, NormalType, DescriptorType>);

	if (k==0){
		  if (pcl::io::loadOBJFile ("/home/idf/ros/catkin_ws/caja.obj", *model) == -1) //* load the file
			    	  {
			    	    PCL_ERROR ("Couldn't read file prueba_captura.pcd \n");
			    	  }
			    	  ROS_INFO_STREAM("Imagen cargada");

/*
			    	  //PRE-PROCESSING (MODEL):

			    	  //passthrough filter
				 	  pass.setInputCloud (model);
				 	  pass.setFilterFieldName ("z");
				 	  pass.setFilterLimits (0.0, 1);
				 	  pass.filter (*model_filtered);
				 	  pass.setInputCloud (model_filtered);
				 	  pass.setFilterFieldName ("x");
				 	  pass.setFilterLimits (-0.25, 0.35);
				 	  pass.filter (*model_filtered);

				 	 //Euclidean cluster extraction (model):
				 	  tree->setInputCloud (model_filtered);

				 	  ec.setClusterTolerance (0.02); // 2cm
				 	  ec.setMinClusterSize (100);
				 	  ec.setMaxClusterSize (25000);
				 	  ec.setSearchMethod (tree);
				 	  ec.setInputCloud (model_filtered);
				 	  ec.extract (cluster_indices_model);

				 	  //Loading cluster indices
				      pcl::IndicesPtr indices_ptr_model (new std::vector<int> (cluster_indices_model[0].indices.size ()));
				      for (int j = 0; j < indices_ptr_model->size(); j++)
				     (*indices_ptr_model)[j] = cluster_indices_model[0].indices[j];

				      std::cout<<"Hasta aquí bien"<<std::endl;
				      std::cout<<indices_ptr_model->size()<<std::endl;
				      std::cout<<model_filtered->size()<<std::endl;

				     //Removing points which don't belong to the main cluster:
				     extract.setInputCloud (model_filtered);
				     extract.setIndices (indices_ptr_model);
				     extract.setNegative (false);
				     extract.filter (*model_filtered);

				     //Downsample
				     pcl::PointIndices::Ptr ind_down_model(new pcl::PointIndices());
				     c=0;
				     for (int k = 0; k < (*model_filtered).size(); k++)
				       {
				    	 if (c==0){
				    		 ind_down_model->indices.push_back(k);
				    		 c=1;
				    	 }
				    	 else
				    		 c=0;
				       }
				     extract2.setInputCloud (model_filtered);
				     extract2.setIndices (ind_down_model);
				     extract2.setNegative (false);
				     extract2.filter (*model_filtered);
*/
					//NORMALS ESTIMATION (MODEL)
					//norm_est.setKSearch (20);
					norm_est.setRadiusSearch(0.025);
					norm_est.setInputCloud (model);
					norm_est.compute (*model_normals);

			    	  //First iteration publishes the model
			    	  pcl::toROSMsg(*model,output);
			    	  output.header=cloud_msg->header;
			    	  k++;


	}

	else{
	  // Convert to PCL data type
	  pcl::fromROSMsg(*cloud_msg, *cloud);


	  std::cout<<"PC original"<<std::endl;
	  std::cout<<cloud->size()<<std::endl;


	  // *********   PRE-PROCESSING: *********
	  //SUMMARY: 1-Work with QQVGA resolution, 2-Passthrough filter removes inf, nan and points located in certain areas, 3-Cluster extraction and removal of uninteresting clusters, 4-Downsampling (50%)
	  //Para QQVGA:

	  //Scene filter (passthrough)
	 	  pass.setInputCloud (cloud);
	 	  pass.setFilterFieldName ("z");
	 	  pass.setFilterLimits (0.6, 1);
	 	  pass.filter (*cloud_filtered);
	 	  pass.setInputCloud (cloud_filtered);
	 	  pass.setFilterFieldName ("x");
	 	  pass.setFilterLimits (-0.25, 0.45);
	 	  pass.filter (*cloud_filtered);


	 	  std::cout<<"PC filtrada"<<std::endl;
	 	  std::cout<<cloud_filtered->size()<<std::endl;


	 	  //Euclidean cluster extraction:

	 	  tree->setInputCloud (cloud_filtered);

	 	  ec.setClusterTolerance (0.02); // 2cm
	 	  ec.setMinClusterSize (100);
	 	  ec.setMaxClusterSize (25000);
	 	  ec.setSearchMethod (tree);
	 	  ec.setInputCloud (cloud_filtered);
	 	  ec.extract (cluster_indices);
	 	 std::cout<<cluster_indices.size()<<std::endl;
	 	  //Loading cluster indices
	      pcl::IndicesPtr indices_ptr (new std::vector<int> (cluster_indices[0].indices.size ()));
	      for (int j = 0; j < indices_ptr->size(); j++)
	     (*indices_ptr)[j] = cluster_indices[0].indices[j];


	      //Removing points which don't belong to the main cluster:
	     extract.setInputCloud (cloud_filtered);
	     extract.setIndices (indices_ptr);
	     extract.setNegative (false);
	     extract.filter (*cloud_filtered);

	 	  std::cout<<"PC pre sample"<<std::endl;
	 	  std::cout<<cloud_filtered->size()<<std::endl;



	     pcl::PointIndices::Ptr ind_down(new pcl::PointIndices());
	     c=0;
	     for (int k = 0; k < (*cloud_filtered).size(); k++)
	       {
	    	 if (c==0){
	    		 ind_down->indices.push_back(k);
	    		 c=1;
	    	 }
	    	 else
	    		 c=0;
	       }
	     extract2.setInputCloud (cloud_filtered);
	     extract2.setIndices (ind_down);
	     extract2.setNegative (false);
	     extract2.filter (*cloud_filtered);



	 	  std::cout<<"PC downsample"<<std::endl;
	 	  std::cout<<cloud_filtered->size()<<std::endl;
	 	  std::cout<<"model downsample"<<std::endl;
	 	  std::cout<<model_filtered->size()<<std::endl;

	 	 // *********   COARSE REGISTRATION: *********


	 	 double cloud_resolution = computeCloudResolution(cloud_filtered);
	 	 std::cout<<cloud_resolution<<std::endl;

	 	 //Normals (Ksearch set in 1st iteration):
		 norm_est.setRadiusSearch(0.025);
		 norm_est.setInputCloud (cloud_filtered);
		 norm_est.compute (*cloud_normals);

		 //ICP
		  pcl::IterativeClosestPoint<PointType, PointType> icp;
		  icp.setInputSource(model);
		  icp.setInputTarget(cloud_filtered);
	//	  icp.setMaximumIterations(50);
		  pcl::PointCloud<PointType> Final;
		  icp.align(Final);
		  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		  icp.getFitnessScore() << std::endl;
		  std::cout << icp.getFinalTransformation() << std::endl;

		  Eigen::Matrix4f transf;
		  transf=icp.getFinalTransformation();

		  std::cout<<transf<<std::endl;
		  pcl::transformPointCloud (*model, *rotated_model, transf);




	  pcl::toROSMsg(*cloud_filtered,output);
	  output.header=cloud_msg->header;
	  pcl::toROSMsg(*model,output2);
	  output2.header=cloud_msg->header;
	  pcl::toROSMsg(*rotated_model,output3);
	  output3.header=cloud_msg->header;
	  k++;

	}


	pub.publish (output);
	pub2.publish(output2);
	pub3.publish(output3);


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
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output3s", 1);



  // Spin
  ros::spin ();
}

