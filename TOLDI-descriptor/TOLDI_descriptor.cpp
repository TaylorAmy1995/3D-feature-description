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
void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,LRF pointLRF,pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud)
{
	pcl::PointXYZ point=cloud->points[0];//the centroid of the local surface
	int number_of_points=cloud->points.size();
	transformed_cloud->points.resize(number_of_points);
	Eigen::Matrix3f matrix;
	matrix(0,0)=pointLRF.x_axis.x;matrix(0,1)=pointLRF.x_axis.y;matrix(0,2)=pointLRF.x_axis.z;
	matrix(1,0)=pointLRF.y_axis.x;matrix(1,1)=pointLRF.y_axis.y;matrix(1,2)=pointLRF.y_axis.z;
	matrix(2,0)=pointLRF.z_axis.x;matrix(2,1)=pointLRF.z_axis.y;matrix(2,2)=pointLRF.z_axis.z;
	for(int i=0;i<number_of_points;i++)
	{
		Eigen::Vector3f transformed_point (
		  cloud->points[i].x - point.x,
		  cloud->points[i].y - point.y,
		  cloud->points[i].z - point.z);

		transformed_point = matrix * transformed_point;

		pcl::PointXYZ new_point;
		new_point.x = transformed_point (0);
		new_point.y = transformed_point (1);
		new_point.z = transformed_point (2);
		transformed_cloud->points[i] = new_point;
	}
}
void local_TOLDI(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float sup_radius,int bin,vector<float>&histogram)
{
	int i,j;
	float step=2*sup_radius/bin;
	int number_of_points=cloud->points.size();
	vector<float>sub_histogram1,sub_histogram2,sub_histogram3;
	//View1
	vector<vector<float>> bin_cloud_LDI1;
	bin_cloud_LDI1.resize(bin*bin);
	for(int i=0;i<number_of_points;i++)
	{
		//calculate bin's index
		float x_bin_temp=(cloud->points[i].x+sup_radius)/step;
		float y_bin_temp=(cloud->points[i].y+sup_radius)/step;
		int x_bin=x_bin_temp+1;
		if(x_bin<1) x_bin=1;
		if(x_bin>bin) x_bin=bin;
		int y_bin=y_bin_temp+1;
		y_bin=bin-y_bin+1;
		if(y_bin<1) y_bin=1;
		if(y_bin>bin) y_bin=bin;
		int bin_Idx=(y_bin-1)*bin+x_bin-1;
		//calculate LDI feature
		float LDI_temp=(sup_radius-cloud->points[i].z)/(2*sup_radius);//Normalization
		if(LDI_temp<0) LDI_temp=0;
		if(LDI_temp>1) LDI_temp=1;

		bin_cloud_LDI1[bin_Idx].push_back(LDI_temp);
	}
	sub_histogram1.resize(bin*bin);
	for(int i=0;i<bin*bin;i++)
	{
		if(bin_cloud_LDI1[i].size()==0) sub_histogram1[i]=TOLDI_NULL_PIXEL;
		else
		{
			sort(bin_cloud_LDI1[i].begin(),bin_cloud_LDI1[i].end());//To resist self-occlusion, take the minimum LDI value as the feature of current bin
			sub_histogram1[i]=bin_cloud_LDI1[i][0];
		}
	}
	//View2
	vector<vector<float>> bin_cloud_LDI2;
	bin_cloud_LDI2.resize(bin*bin);
	for(int i=0;i<number_of_points;i++)
	{
		float x_bin_temp=(cloud->points[i].y+sup_radius)/step;
		float y_bin_temp=(cloud->points[i].z+sup_radius)/step;
		int x_bin=x_bin_temp+1;
		if(x_bin<1) x_bin=1;
		if(x_bin>bin) x_bin=bin;
		int y_bin=y_bin_temp+1;
		y_bin=bin-y_bin+1;
		if(y_bin<1) y_bin=1;
		if(y_bin>bin) y_bin=bin;
		int bin_Idx=(y_bin-1)*bin+x_bin-1;

		float LDI_temp=(sup_radius-cloud->points[i].x)/(2*sup_radius);
		if(LDI_temp<0) LDI_temp=0;
		if(LDI_temp>1) LDI_temp=1;

		bin_cloud_LDI2[bin_Idx].push_back(LDI_temp);
	}
	sub_histogram2.resize(bin*bin);
	for(int i=0;i<bin*bin;i++)
	{
		if(bin_cloud_LDI2[i].size()==0) sub_histogram2[i]=TOLDI_NULL_PIXEL;
		else
		{
			sort(bin_cloud_LDI2[i].begin(),bin_cloud_LDI2[i].end());
			sub_histogram2[i]=bin_cloud_LDI2[i][0];
		}
	}
	//View3
	vector<vector<float>> bin_cloud_LDI3;
	bin_cloud_LDI3.resize(bin*bin);
	for(int i=0;i<number_of_points;i++)
	{

		float x_bin_temp=(cloud->points[i].z+sup_radius)/step;
		float y_bin_temp=(cloud->points[i].x+sup_radius)/step;
		int x_bin=x_bin_temp+1;
		if(x_bin<1) x_bin=1;
		if(x_bin>bin) x_bin=bin;
		int y_bin=y_bin_temp+1;
		y_bin=bin-y_bin+1;
		if(y_bin<1) y_bin=1;
		if(y_bin>bin) y_bin=bin;
		int bin_Idx=(y_bin-1)*bin+x_bin-1;

		float LDI_temp=(sup_radius-cloud->points[i].y)/(2*sup_radius);
		if(LDI_temp<0) LDI_temp=0;
		if(LDI_temp>1) LDI_temp=1;

		bin_cloud_LDI3[bin_Idx].push_back(LDI_temp);
	}
	sub_histogram3.resize(bin*bin);
	for(int i=0;i<bin*bin;i++)
	{
		if(bin_cloud_LDI3[i].size()==0) sub_histogram3[i]=TOLDI_NULL_PIXEL;
		else
		{
			sort(bin_cloud_LDI3[i].begin(),bin_cloud_LDI3[i].end());
			sub_histogram3[i]=bin_cloud_LDI3[i][0];
		}
	}
	//Concatenate the three sub-histograms, generating the final TOLDI feature
	std::copy(sub_histogram2.begin(),sub_histogram2.end(),std::back_inserter(sub_histogram1));
	std::copy(sub_histogram3.begin(),sub_histogram3.end(),std::back_inserter(sub_histogram1));
	histogram=sub_histogram1;
}
void TOLDI_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int>indices,float sup_radius,int bin_num,vector<vector<float>>&Histograms)
{
	int i,j,m;
	//kdtree-search
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int>pointIdx;
	vector<float>pointDst;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ query_point;

	for(i=0;i<indices.size();i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor(new pcl::PointCloud<pcl::PointXYZ>);//local surface
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_z(new pcl::PointCloud<pcl::PointXYZ>);//local surface for computing the z-axis of LRF
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_trans(new pcl::PointCloud<pcl::PointXYZ>);//transformed local surface w.r.t. to LRF
		Vertex x_axis,y_axis,z_axis;
		query_point=cloud->points[indices[i]];
		//
		if(kdtree.radiusSearch(query_point,sup_radius/3,pointIdx,pointDst)>3)
		{
			for(j=0;j<pointIdx.size();j++)
			{
				sphere_neighbor_z->points.push_back(cloud->points[pointIdx[j]]);
			}
			TOLDI_LRF_Z_axis(sphere_neighbor_z,z_axis);
			if(kdtree.radiusSearch(query_point,sup_radius,pointIdx,pointDst)>10)//only if there are more than 10 points in the local surface
			{
				for(j=0;j<pointIdx.size();j++)
				{
					sphere_neighbor->points.push_back(cloud->points[pointIdx[j]]);
				}
				TOLDI_LRF_X_axis(sphere_neighbor,z_axis,sup_radius,pointDst,x_axis);
				TOLDI_LRF_Y_axis(x_axis,z_axis,y_axis);
				LRF pointLRF={indices[i],x_axis,y_axis,z_axis};
				transformCloud(sphere_neighbor,pointLRF,sphere_neighbor_trans);//transform the local surface w.r.t. TOLDI-LRF
				vector<float> TriLDI_feature;
				local_TOLDI(sphere_neighbor_trans,sup_radius,bin_num,TriLDI_feature);
				Histograms.push_back(TriLDI_feature);
			}
			else
			{
				vector<float> f_default(3*bin_num*bin_num,0.0f);
				Histograms.push_back(f_default);
			}
		}
		else
		{
			vector<float> f_default(3*bin_num*bin_num,0.0f);
			Histograms.push_back(f_default);
		}
	}
}