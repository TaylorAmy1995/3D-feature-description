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
void TOLDI_LRF_Z_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Vertex &z_axis)
{
	int i;
	pcl::PointXYZ query_point=cloud->points[0];
	// calculate covariance matrix
	Eigen::Matrix3f Cov;
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud,centroid);
	pcl::computeCovarianceMatrix(*cloud,centroid,Cov);
	EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_min;
    EIGEN_ALIGN16 Eigen::Vector3f normal;
	pcl::eigen33(Cov,eigen_min,normal);
	z_axis.x=normal(0);
	z_axis.y=normal(1);
	z_axis.z=normal(2);
	// z-axis sign disambiguity
	float z_sign=0;
	for(i=0;i<cloud->points.size();i++)
	{
		float vec_x=query_point.x-cloud->points[i].x;
		float vec_y=query_point.y-cloud->points[i].y;
		float vec_z=query_point.z-cloud->points[i].z;
		z_sign+=(vec_x*z_axis.x+vec_y*z_axis.y+vec_z*z_axis.z);
	}
	if(z_sign<0)
	{
		z_axis.x=-z_axis.x;
		z_axis.y=-z_axis.y;
		z_axis.z=-z_axis.z;
	}
}
void TOLDI_LRF_X_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Vertex z_axis,float sup_radius,vector<float> PointDist,Vertex &x_axis)
{
	int i,j;
	pcl::PointXYZ query_point=cloud->points[0];
	//
	vector<Vertex> vec_proj;
	vector<float>dist_weight,sign_weight;//store weights w1,w2
	for(i=0;i<cloud->points.size();i++)
	{
		Vertex temp;
		Vertex pq={cloud->points[i].x-query_point.x,cloud->points[i].y-query_point.y,cloud->points[i].z-query_point.z};//pq向量为邻域点与查询点组成的向量
		float proj=z_axis.x*pq.x+z_axis.y*pq.y+z_axis.z*pq.z;
		if(proj>=0)
			sign_weight.push_back(pow(proj,2));
		else
			sign_weight.push_back(-pow(proj,2));
		temp.x=pq.x-proj*z_axis.x;
		temp.y=pq.y-proj*z_axis.y;
		temp.z=pq.z-proj*z_axis.z;
		vec_proj.push_back(temp);
	}
	
	for(i=0;i<PointDist.size();i++)
	{
		float wei_temp=sup_radius-sqrt(PointDist[i]);
		wei_temp=pow(wei_temp,2);
		dist_weight.push_back(wei_temp);
	}
	Vertex x_axis_temp={0.0f,0.0f,0.0f};
	for(i=0;i<cloud->points.size();i++)
	{
		float weight_sum=dist_weight[i]*sign_weight[i];
		x_axis_temp.x+=weight_sum*vec_proj[i].x;
		x_axis_temp.y+=weight_sum*vec_proj[i].y;
		x_axis_temp.z+=weight_sum*vec_proj[i].z;
	}
	//Normalization
	float size=sqrt(pow(x_axis_temp.x,2)+pow(x_axis_temp.y,2)+pow(x_axis_temp.z,2));
	x_axis_temp.x/=size;x_axis_temp.y/=size;x_axis_temp.z/=size;
	x_axis=x_axis_temp;
}
void TOLDI_LRF_Y_axis(Vertex x_axis,Vertex z_axis,Vertex &y_axis)
{
	Eigen::Vector3f x(x_axis.x,x_axis.y,x_axis.z);
	Eigen::Vector3f z(z_axis.x,z_axis.y,z_axis.z);
	Eigen::Vector3f y;
	y=x.cross(z);//cross product
	y_axis.x=y(0);
	y_axis.y=y(1);
	y_axis.z=y(2);
}
void TOLDI_LRF_for_cloud_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int> indices,float sup_radius,vector<LRF>&Cloud_LRF)
{
	int i,j,m;
	//
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int>pointIdx;
	vector<float>pointDst;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ query_point;
	//LRF calculation
	for(i=0;i<indices.size();i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor(new pcl::PointCloud<pcl::PointXYZ>);//local surface
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_z(new pcl::PointCloud<pcl::PointXYZ>);//local surface for computing the z-axis of LRF
		query_point=cloud->points[indices[i]];
		//
		if(kdtree.radiusSearch(query_point,sup_radius/3,pointIdx,pointDst)>3)
		{
			for(j=0;j<pointIdx.size();j++)
			{
				sphere_neighbor_z->points.push_back(cloud->points[pointIdx[j]]);
			}
			if(kdtree.radiusSearch(query_point,sup_radius,pointIdx,pointDst)>10)//only if there are more than 10 points in the local surface
			{
				for(j=0;j<pointIdx.size();j++)
				{
					sphere_neighbor->points.push_back(cloud->points[pointIdx[j]]);
				}
				Vertex x_axis,y_axis,z_axis;
				TOLDI_LRF_Z_axis(sphere_neighbor_z,z_axis);
				TOLDI_LRF_X_axis(sphere_neighbor,z_axis,sup_radius,pointDst,x_axis);
				TOLDI_LRF_Y_axis(x_axis,z_axis,y_axis);
				LRF temp={indices[i],x_axis,y_axis,z_axis};
				Cloud_LRF.push_back(temp);
			}
			else
			{
				LRF temp={NULL_POINTID,{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
				Cloud_LRF.push_back(temp);
			}
		}
		else
		{
			LRF temp={NULL_POINTID,{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};
			Cloud_LRF.push_back(temp);
		}
	}
}