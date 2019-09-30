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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <chrono>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

typedef pcl::PointNormal NormalType;
typedef pcl::PointXYZ PointType;

pcl::PointCloud<NormalType>::Ptr adjusted_model (new pcl::PointCloud<NormalType> ());
pcl::PointCloud<NormalType>::Ptr initial_model (new pcl::PointCloud<NormalType> ());

geometry_msgs::Point closest_point;
geometry_msgs::Point closest_normal;
geometry_msgs::Pose aux_point;
geometry_msgs::Pose aux_norm;


tf::StampedTransform transform;
Eigen::Matrix4f eigen_transform, eigen_transform_ant, subs;
float x,y, z, dif, dif_1, x_m, y_m,z_m, final_x,final_y,final_z, init_x, init_y, init_z, n_x,n_y,n_z, norm_vector, xt,yt,zt,xt1,yt1,zt1, fit_prev;
int ind, ind2;
std::vector<int> ind_list;
int count=0;
int do_tr=0;
int num_el;
bool new_coords=false;
bool new_transform=false;
bool first_transform=true;
bool first_coord=false;
bool good_transform=false;
bool new_list=false;
bool first_final_list=false;
bool temp_match=false;
Eigen::Matrix3f m;
Eigen::Matrix4f trasl, trtotal, transf_total;
Eigen::MatrixXf trot(4,4);

ros::Publisher pub_point;
ros::Publisher pub_normal;

ros::Publisher pub_m;
ros::Publisher pub_list;
ros::Publisher pub_ifnewlist;
sensor_msgs::PointCloud2 output_merged;
geometry_msgs::PoseArray polishing_list;
geometry_msgs::PoseArray transformed_list;
std_msgs::Bool ifnewlist;


void coords_cb (geometry_msgs::Point coords){
	x=coords.x;
	y=coords.y;
	z=coords.z;
	new_coords=true;
}

void fit_cb (std_msgs::Int32 do_msg){
	if(do_msg.data==1){
		good_transform=true;
	}
	else{
		good_transform=false;
	}
}

void list_cb (geometry_msgs::PoseArray list){
	std::cout<<"hola"<<std::endl;
	polishing_list=list;
	new_list=true;
}

int
main (int argc, char** argv)
{
	  ros::init (argc, argv, "find_points_normals_control");
	  ros::NodeHandle nh;

	  pub_point = nh.advertise<geometry_msgs::Point> ("closest_point", 1);
	  pub_normal= nh.advertise<geometry_msgs::Point> ("closest_normal",1);
	  pub_m = nh.advertise<sensor_msgs::PointCloud2> ("output_merged", 1);
	  pub_list= nh.advertise<geometry_msgs::PoseArray>("final_list",1);
	  pub_ifnewlist= nh.advertise<std_msgs::Bool>("new_list",1);


	  ros::Subscriber sub2 = nh.subscribe("/input_coords",1,coords_cb);
	  ros::Subscriber sub_do_tr=nh.subscribe("/good_tr",1,fit_cb);
	  ros::Subscriber sub_list = nh.subscribe("/polishing_list",1,list_cb);
	  //How to publish: 'rostopic pub /input_coords geometry_msgs/Points {x_value,y_value,z_value}'


	  if (pcl::io::loadOBJFile ("/home/idf/ros_ws/kuka_ws/src/pcl_icp_matching3d/Models/car_pruebav2.obj", *initial_model) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read file car_door.obj \n");
	  }

	  // BLOQUE PROVISIONAL, BORRAR CUANDO YA ESTÉ EN FUNCIONAMIENTO EL CONJUNTO ENTERO:
	  m=Eigen::AngleAxisf(90*M_PI/180.0,Eigen::Vector3f::UnitZ())*Eigen::AngleAxisf(-5.0*M_PI/180.0,Eigen::Vector3f::UnitY());
	  trot.block<3,3>(0,0)=m;
	  trot(0,3)=0;
	  trot(1,3)=0;
	  trot(2,3)=0;
	  trot(3,3)=1;
	  trot(3,0)=0;
	  trot(3,1)=0;
	  trot(3,2)=0;
	  trasl<<1, 0, 0, 0.6,
			0, 1, 0, 0.3,
			0, 0, 1, 0.7,
			0, 0, 0, 1;
	  trtotal=trasl*trot;
	  transf_total.setIdentity();
	  pcl::transformPointCloudWithNormals (*initial_model, *adjusted_model, trtotal);
	  new_transform=true;
	  first_transform=false;
	  good_transform=true;
	  ///////////////////////////////////////////////////////////////////////////////////

	  ros::Rate rate(100);
	  while(ros::ok())
	  {

		  if(new_coords==true && first_transform==false){
		  dif_1=1000000;
//		  auto start = std::chrono::high_resolution_clock::now();
		  for(int j=0;j<(*adjusted_model).size();j++){
			  x_m=(*adjusted_model).points[j].x;
			  y_m=(*adjusted_model).points[j].y;
			  z_m=(*adjusted_model).points[j].z;
			  dif=pow((x-x_m),2)+pow((y-y_m),2)+pow((z-z_m),2);
			  if(dif<dif_1){
				  ind=j;
				  dif_1=dif;
			  }
		  }

		  init_x=(*adjusted_model).points[ind].x;
		  init_y=(*adjusted_model).points[ind].y;
		  init_z=(*adjusted_model).points[ind].z;
		  n_x=(*adjusted_model).points[ind].normal_x;
		  n_y=(*adjusted_model).points[ind].normal_y;
		  n_z=(*adjusted_model).points[ind].normal_z;
		  new_coords=false;
		  first_coord=true;
		  }

		  if(new_list==true && first_transform==false){
			  num_el=polishing_list.poses.size();
			  ind_list.clear();
			  //ind_list.resize(num_el);
			  Eigen::MatrixXf pol_mat(num_el,3);
			  for(int j=0;j<num_el;j++){
				  pol_mat(j,0)=polishing_list.poses[j].position.x;
				  pol_mat(j,1)=polishing_list.poses[j].position.y;
				  pol_mat(j,2)=polishing_list.poses[j].position.z;
			  }


			  for (int k=0;k<num_el;k++){
				  dif_1=10000;
			  for(int j=0;j<(*initial_model).size();j++){
			  	x_m=(*initial_model).points[j].x;
			  	y_m=(*initial_model).points[j].y;
			  	z_m=(*initial_model).points[j].z;
			  	dif=pow((pol_mat(k,0)-x_m),2)+pow((pol_mat(k,1)-y_m),2)+pow((pol_mat(k,2)-z_m),2);
			  	if(dif<dif_1){
			  		ind2=j;
			  		dif_1=dif;
			  }
			 }
			  std::cout<<(*initial_model).points[ind2].x<<" "<<(*initial_model).points[ind2].y<<" "<<(*initial_model).points[ind2].z<<std::endl;
			  std::cout<<ind2<<std::endl;
			  std::cout<<dif_1<<std::endl;
			  ind_list.push_back(ind2);
			  }

			first_final_list=true;
			std::cout<<ind_list[0]<<" "<<ind_list[1]<<" "<<ind_list[2]<<std::endl;
			 }


		  if(first_coord==true){
			  closest_point.x=init_x;
			  closest_point.y=init_y;
			  closest_point.z=init_z;
			  closest_normal.x=n_x;
			  closest_normal.y=n_y;
			  closest_normal.z=n_z;

			  pub_point.publish(closest_point);
			  pub_normal.publish(closest_normal);


		  }

		  if(first_final_list==true && new_transform==true && good_transform==true){
			  std::cout<<"hasta aquí antes"<<std::endl;
			  transformed_list.poses.clear();

		  for(int k=0;k<num_el;k++){

			  aux_point.position.x=(*adjusted_model).points[ind_list[k]].x;
			  aux_point.position.y=(*adjusted_model).points[ind_list[k]].y;
			  aux_point.position.z=(*adjusted_model).points[ind_list[k]].z;

			  aux_norm.position.x=(*adjusted_model).points[ind_list[k]].normal_x;
			  aux_norm.position.y=(*adjusted_model).points[ind_list[k]].normal_y;
			  aux_norm.position.z=(*adjusted_model).points[ind_list[k]].normal_z;


			  transformed_list.poses.push_back(aux_point);
			  transformed_list.poses.push_back(aux_norm);
		  }
		  std::cout<<"hasta aquí"<<std::endl;

		  pub_list.publish(transformed_list);
		  }
		  // BLOQUE PROVISIONAL, BORRAR CUANDO YA ESTÉ EN FUNCIONAMIENTO EL CONJUNTO ENTERO:
		  pcl::toROSMsg(*adjusted_model,output_merged);
		  output_merged.header.frame_id="/world";
		  pub_m.publish(output_merged);
		  /////////////////////////////////////////////////////////////////////////////



		  ifnewlist.data=new_list;
		  pub_ifnewlist.publish(ifnewlist);
		  if(new_list==true){

		  new_list=false;
		  }
		  // Spin
		  rate.sleep();
		  ros::spinOnce();
	  }

}
