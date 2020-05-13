/******************************Ref. Code for computing a set of RCS (float and binary) features for a point cloud*********************************/
Please install the Point Cloud Library (PCL, http://pointclouds.org/downloads/) first, we use the PCL 1.6.0 (or higher versions) in RCS implementation

1. We provide interfaces to compute RCS features, including both float and real-valued ones.

2. void RCS_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points, vector<vector<float>>&Histograms)
~cloud: the input point cloud, refer to the pcl::io module to load a point cloud from .xyz or .ply files
~indices: the keypoint indices
~sup_radius: the support radius of the sphere used to crop the local surface from a point cloud
~rotate_angle: unit is degree, it means the angles set for each local surface rotation around x/y/z axis in the local reference frame (LRF), we use 30 degrees;
~num_of_rotations: the number of rotations used for multi-view information, we use 6;
~num_of_contour_points: the number of contour points in each signature of RCS, we use 12;
~Histograms: the final RCS float features, 1 RCS feature for 1 keypoint

3. The explanations for RCS binary features are almost identical to the above function. The difference is only the data type of the output feature.

4. Please cite the following article if you use the ref. code:
 Jiaqi Yang, et al. "Rotational contour signatures for robust local surface description", ICIP 2016.
 Jiaqi Yang, et al. "Rotational contour signatures for both real-valued and binary feature representations of 3D local shape", CVIU 2017.
 
 5. Report bugs to: jqyang@hust.edu.cn
 
