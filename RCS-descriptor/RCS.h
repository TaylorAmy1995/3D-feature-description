#ifndef _RCS__H_ 
#define _RCS__H_
#define Pi 3.1415926
//
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointXYZ PointInT;
typedef struct{
	float x;
	float y;
	float z;
}Vertex;
typedef struct{
	float x;
	float y;
	float z;
	float dist;
	float angle_to_axis;
}Vertex_d_ang;
typedef struct{
	int pointID;
	float proj_dist;
}Board_element;
typedef struct{
	int pointID;
	Vertex x_axis;
	Vertex y_axis;
	Vertex z_axis;
}LRF;
//

/*************************************************RCS_float****************************************************/
//RCS
void LRF_Z_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Vertex &z_axis);
void LRF_X_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Vertex z_axis,float sup_radius,vector<float>  PointDist,Vertex &x_axis);
void LRF_axis(Vertex x_axis,Vertex z_axis,Vertex &y_axis);
void VectorNormlize(Vertex& vec);
void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,LRF pointLRF,pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud);
void RotateCloud(const LRF pointLRF,const float angle,const PointCloudPtr cloud,PointCloudPtr&rotated_cloud);
bool RCS_SortBydist(const Vertex_d_ang v1,const Vertex_d_ang v2);
void RCS_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,vector<float>&signature);
void RCS_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points, vector<vector<float>>&Histograms);
/*************************************************Binary descriptor****************************************************/
//RCS_B-thresholding:Max_entropy
void RCS_B_thresholding_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,vector<char>&signature);
void RCS_B_thresholding_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points,vector<vector<char>>&Histograms);

//RCS_B-quant
void RCS_B_quant_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,int num_quant_bit,vector<char>&signature);
void RCS_B_quant_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int>indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points,int num_quant_bit,vector<vector<char>>&Histograms);
//RCS_B-geometry
void RCS_B_geometry_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,vector<char>&signature);
void RCS_B_geometry_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points,vector<vector<char>>&Histograms);
#endif