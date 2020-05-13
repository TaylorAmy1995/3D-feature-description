#include <stdio.h>
#include <vector>
#include <time.h>
#include <pcl\point_types.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/eigen.h>
#include "RCS.h"
int main(int argc, char *argv[])
{
	/*You need to first load a point cloud from disk, try pcl::io module*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	/*Then, You need to determine the point index, i.e., keypoints, for RCS extraction.  Put them in a vector with int type.*/
	vector<int> indices;
	////////////////
	float sup_radius;//decide it according to your data scale
	float rotate_angle=30;
	int num_of_rotations=6,num_of_contour_points=12;
	vector<vector<float>>RCS_float_features;
	vector<vector<char>>RCS_binary_features;
	//examples for RCS_float and RCS_binary features
	RCS_compute(cloud,indices, sup_radius, rotate_angle, num_of_rotations, num_of_contour_points, RCS_float_features);
	RCS_B_geometry_compute(cloud,indices, sup_radius, rotate_angle, num_of_rotations, num_of_contour_points, RCS_binary_features);
	return 0;
}
