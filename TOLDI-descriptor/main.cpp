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
#include "TOLDI.h"
int XYZorPly_Read(string Filename,PointCloudPtr&cloud)
{
	int i,j,k;
	int nXYZ_nums;
	std::vector<Vertex> vXYZ;
	FILE *fp = fopen(Filename.c_str(),"r");
	if(fp==NULL)
	{
		printf("File can't open!\n");
		return -1;
	}
	const char*FILEPATH=Filename.c_str();
	char a=FILEPATH[strlen(FILEPATH)-1];
	//
	if(a=='y')
	{
		char str[1024];
		fscanf(fp,"%s\n",&str);
		fscanf(fp,"%s %s %s\n",&str,&str,&str);
		fscanf(fp,"%s %s %d\n",&str,&str,&nXYZ_nums);
		fscanf(fp,"%s %s %s\n",&str,&str,&str);
		fscanf(fp,"%s %s %s\n",&str,&str,&str);
		fscanf(fp,"%s %s %s\n",&str,&str,&str);
		fscanf(fp,"%s %s %s\n",&str,&str,&str);
		fscanf(fp,"%s %s %s %s %s\n",&str,&str,&str,&str,&str);
		fscanf(fp,"%s\n",&str);
	}
	else
	{
		fscanf(fp,"%d\n",&nXYZ_nums);
	}
	vXYZ.resize(nXYZ_nums);
	for (i = 0; i < vXYZ.size(); i++)
	{
		fscanf(fp,"%f %f %f\n", &vXYZ[i].x, &vXYZ[i].y, &vXYZ[i].z);
	}
	fclose(fp);
	cloud->width = vXYZ.size();
	cloud->height = 1;
	cloud->is_dense = true;
	cloud->points.resize(cloud->width*cloud->height);
	for (i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x = vXYZ[i].x;
		cloud->points[i].y = vXYZ[i].y;
		cloud->points[i].z = vXYZ[i].z;
	}
	return 0;
}
int main(int argc, char *argv[])
{
	string cloud_file=argv[1];
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	XYZorPly_Read(cloud_file,cloud);
	//TOLDI_LRF_for_cloud_compute(cloud, indices,sup_radius,Cloud_LRF);
	//TOLDI_compute(cloud,indices,sup_radius,bin_num,TOLDI_features);
	return 0;
}