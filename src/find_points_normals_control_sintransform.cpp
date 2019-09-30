#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>
#include <iostream>
#include <pcl/common/transforms.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <chrono>

typedef pcl::PointNormal NormalType;
typedef pcl::PointXYZ PointType;

pcl::PointCloud<NormalType>::Ptr adjusted_model (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr initial_model (new pcl::PointCloud<NormalType> ());

visualization_msgs::Marker normal;
geometry_msgs::Point point_init;
geometry_msgs::Point point_end;

tf::StampedTransform transform;
Eigen::Matrix4f eigen_transform;
float x,y, z, dif, dif_1, x_m, y_m,z_m, final_x,final_y,final_z, init_x, init_y, init_z, n_x,n_y,n_z, norm_vector;
int ind;
bool new_coords=false;

ros::Publisher pub;


void coords_cb (geometry_msgs::Point coords){
	x=coords.x;
	y=coords.y;
	z=coords.z;
	new_coords=true;
}

int
main (int argc, char** argv)
{
	  ros::init (argc, argv, "find_points_normals_control");
	  ros::NodeHandle nh;

	  pub = nh.advertise<visualization_msgs::Marker> ("normal_arrow", 1);

	  ros::Subscriber sub2 = nh.subscribe("/input_coords",1,coords_cb);
	  //How to publish: 'rostopic pub /input_coords geometry_msgs/Points {x_value,y_value,z_value}'

	  normal.header.frame_id="/world";
	  normal.type=visualization_msgs::Marker::ARROW;
	  normal.color.r=0.0;
	  normal.color.g=0.0;
	  normal.color.b=1.0;
	  normal.color.a=1.0;

	  if (pcl::io::loadOBJFile ("/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/car_prueba.obj", *initial_model) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read file car_door.obj \n");
	  }

	  ros::Rate rate(100);
	  while(ros::ok())
	  {

//		  std::cout<<"X coordinate:"<<std::endl;
//		  std::cin>>x;
//		  std::cout<<"Y coordinate:"<<std::endl;
//		  std::cin>>y;
		  if(new_coords==true){
		  dif_1=1000000;
		  auto start = std::chrono::high_resolution_clock::now();
		  for(int j=0;j<(*initial_model).size();j++){
			  x_m=(*initial_model).points[j].x;
			  y_m=(*initial_model).points[j].y;
			  z_m=(*initial_model).points[j].z;
			  dif=pow((x-x_m),2)+pow((y-y_m),2)+pow((z-z_m),2);
			  if(dif<dif_1){
				  ind=j;
				  dif_1=dif;
			  }
		  }
		  auto finish = std::chrono::high_resolution_clock::now();
		  std::chrono::duration<double> elapsed = finish - start;
		  std::cout<<elapsed.count()<<std::endl;
		  std::cout<<(*initial_model).points[ind].x<<" "<<(*initial_model).points[ind].y<<" "<<(*initial_model).points[ind].z<<std::endl;
		  new_coords=false;
		  }


//
//		  pcl_ros::transformAsMatrix (transform, eigen_transform);
//		  pcl::transformPointCloudWithNormals (*initial_model, *adjusted_model, eigen_transform);
//		 // std::cout<<eigen_transform<<std::endl;
//
//		  std::cout<<(*adjusted_model).points[ind].x<<" "<<(*adjusted_model).points[ind].y<<" "<<(*adjusted_model).points[ind].z<<std::endl;
//		  std::cout<<(*adjusted_model).points[ind].normal_x<<" "<< (*adjusted_model).points[ind].normal_y<<" "<<(*adjusted_model).points[ind].normal_z<<std::endl;
//
//		  std::cout<<ind<<std::endl;
//		  //configuring arrow for visualization of normal at selected point:
//		  init_x=(*adjusted_model).points[ind].x;
//		  init_y=(*adjusted_model).points[ind].y;
//		  init_z=(*adjusted_model).points[ind].z;
//		  n_x=(*adjusted_model).points[ind].normal_x;
//		  n_y=(*adjusted_model).points[ind].normal_y;
//		  n_z=(*adjusted_model).points[ind].normal_z;
//		  final_x=init_x+0.1*n_x;
//		  final_y=init_y+0.1*n_y;
//		  final_z=init_z+0.1*n_z;
//		  point_init.x=init_x;
//		  point_init.y=init_y;
//		  point_init.z=init_z;
//		  point_end.x=final_x;
//		  point_end.y=final_y;
//		  point_end.z=final_z;
//		  normal.points.clear();
//		  normal.points.push_back(point_init);
//		  normal.points.push_back(point_end);
//		  normal.scale.x=0.005;
//		  normal.scale.y=0.01;
//		  normal.scale.z=0.002;
//		  pub.publish(normal);

		  // Spin
		  rate.sleep();
		  ros::spinOnce();
	  }

}
