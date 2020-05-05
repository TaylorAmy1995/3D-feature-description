/******************************Ref. Code for computing a set of TOLDI features for a point cloud*********************************/
Please install the Point Cloud Library (PCL, http://pointclouds.org/downloads/) first, we use the PCL 1.6.0 in TOLDI implementation

1. There are two major C functions in our codes, i.e., TOLDI-LRF and TOLDI-descriptor, note that if you just want to extact TOLDI features, ignore
 the TOLDI-LRF function because TOLDI include the TOLDI-LRF calculation process;
2.  void TOLDI_LRF_for_cloud_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int> indices,float sup_radius,vector<LRF>&Cloud_LRF);
~cloud: the input point cloud, refer to the XYZ_or_Ply function in .main file to load a point cloud from .xyz or .ply files
~indices: the keypoint indices
~sup_radius: the support radius of the sphere used to crop the local surface from a point cloud
~Cloud_LRF: the LRFs calculated for the keypoints;
3. void TOLDI_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int>indices,float sup_radius,int bin_num,vector<vector<float>>&Histograms);
~cloud: the input point cloud, refer to the XYZ_or_Ply function in .main file to load a point cloud from .xyz or .ply files
~indices: the keypoint indices
~sup_radius: the support radius of the sphere used to crop the local surface from a point cloud
~bin_num: the parameter of TOLDI, we set it as 20 in our paper
~Histograms: the TOLDI features calculated for the keypoints;
4. Please cite the following article if you use the ref. code:
 "TOLDI: an effective and robust approach for 3D local shape description"