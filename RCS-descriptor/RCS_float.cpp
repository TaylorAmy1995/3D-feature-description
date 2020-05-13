#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/registration/transforms.h>
#include <stdio.h>
#include <math.h>
#include "RCS.h"
/**************************************************RCS***********************************************/
void LRF_Z_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Vertex &z_axis)
{
	int i;
	pcl::PointXYZ query_point=cloud->points[0];

	// Covariance matrix
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
	// disambiguity the sign of Z-axis
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
void LRF_X_axis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Vertex z_axis,float sup_radius,vector<float>  PointDist,Vertex &x_axis)//TOLDI method for x_axis
{
	int i,j;
	pcl::PointXYZ query_point=cloud->points[0];
	//
	vector<Vertex> vec_proj;
	vector<float>dist_weight,sign_weight;//two weights for each projection vector
	for(i=0;i<cloud->points.size();i++)
	{
		Vertex temp;
		Vertex pq={cloud->points[i].x-query_point.x,cloud->points[i].y-query_point.y,cloud->points[i].z-query_point.z};
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
	//normalize
	float size=sqrt(pow(x_axis_temp.x,2)+pow(x_axis_temp.y,2)+pow(x_axis_temp.z,2));
	x_axis_temp.x/=size;x_axis_temp.y/=size;x_axis_temp.z/=size;
	x_axis=x_axis_temp;
}
void LRF_axis(Vertex x_axis,Vertex z_axis,Vertex &y_axis)
{
	Eigen::Vector3f x(x_axis.x,x_axis.y,x_axis.z);
	Eigen::Vector3f z(z_axis.x,z_axis.y,z_axis.z);
	Eigen::Vector3f y;
	y=x.cross(z);
	y_axis.x=y(0);
	y_axis.y=y(1);
	y_axis.z=y(2);
}
void VectorNormlize(Vertex& vec)
{
	float length=sqrt(pow(vec.x,2)+pow(vec.y,2)+pow(vec.z,2));
	vec.x/=length;
	vec.y/=length;
	vec.z/=length;
}
void transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,LRF pointLRF,pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud)
{
	pcl::PointXYZ point=cloud->points[0];//the keypoint
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
void RotateCloud(const LRF pointLRF,const float angle,const PointCloudPtr cloud,PointCloudPtr&rotated_cloud)
{
	Eigen::Matrix4f rotation_matrix;
	float sx,sy,sz,cx,cy,cz;
	sx=(angle/180)*Pi;sy=(angle/180)*Pi;sz=(angle/180)*Pi;
	cx=sx;cy=sy;cz=sz;
	sx=sin(sx);sy=sin(sy);sz=sin(sz);cx=cos(cx);cy=cos(cy);cz=cos(cz);
	//
	rotation_matrix(0,0)=cy*cz;rotation_matrix(0,1)=cy*sz;rotation_matrix(0,2)=-sy;rotation_matrix(0,3)=0;
	rotation_matrix(1,0)=sx*sy*cz-cx*sz;rotation_matrix(1,1)=sx*sy*sz+cx*cz;rotation_matrix(1,2)=sx*cy;rotation_matrix(1,3)=0;
	rotation_matrix(2,0)=cx*sy*cz+sx*sz;rotation_matrix(2,1)=cx*sy*sz-sx*cz;rotation_matrix(2,2)=cx*cy;rotation_matrix(2,3)=0;
	rotation_matrix(3,0)=0;rotation_matrix(3,1)=0;rotation_matrix(3,2)=0;rotation_matrix(3,3)=1;
	//
	pcl::transformPointCloud (*cloud,*rotated_cloud,rotation_matrix);
}
bool RCS_SortBydist(const Vertex_d_ang v1,const Vertex_d_ang v2)
{
	return v1.dist<v2.dist;//ascending
}
void RCS_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,vector<float>&signature)
{
	int i,j;
	vector<Vertex_d_ang> rotated_points;
	//compute the attribute per point
	for(i=0;i<rotated_cloud->points.size();i++)
	{
		float dist=sqrt(rotated_cloud->points[i].x*rotated_cloud->points[i].x+rotated_cloud->points[i].y*rotated_cloud->points[i].y);
		float ang;
		float abs_sin;
		if(dist==0)
			abs_sin=0;
		else
			abs_sin=abs(rotated_cloud->points[i].x)/dist;
		if(abs_sin>1) abs_sin=1;
		if(abs_sin+1<0) abs_sin=-0.9999;//avoid sin errors
		float abs_ang=asin(abs_sin)*180/Pi;
		if((rotated_cloud->points[i].x>=0)&&(rotated_cloud->points[i].y>=0)) ang=abs_ang;//1 quadrant
		else if((rotated_cloud->points[i].x>=0)&&(rotated_cloud->points[i].y<0)) ang=360-abs_ang;//4 quadrant
		else if((rotated_cloud->points[i].x<0)&&(rotated_cloud->points[i].y<0)) ang=180+abs_ang;//3 quadrant
		else ang=180-abs_ang;//2 quadrant
		Vertex_d_ang temp={rotated_cloud->points[i].x,rotated_cloud->points[i].y,rotated_cloud->points[i].z,dist,ang};
		rotated_points.push_back(temp);
	}
	//compute contour points
	float ang_step=360/num_of_contour_points;
	float ang_thresh=15;//this thresh is used to find the point set in the projected point map for approximately determing a contour point,  degrees
	vector<vector<Vertex_d_ang>> point_clusters;
	point_clusters.resize(num_of_contour_points);
	for(i=0;i<rotated_points.size();i++)
	{
		int mod=int(rotated_points[i].angle_to_axis)%int(ang_step);
		int mul=int(rotated_points[i].angle_to_axis)/int(ang_step);
		if((mod<=ang_thresh)&&(mul<num_of_contour_points))
			point_clusters[mul].push_back(rotated_points[i]);
	}
	//compute coutour signature
	signature.resize(num_of_contour_points);
	for(i=0;i<point_clusters.size();i++)
	{
		if(point_clusters[i].size()==0)
			signature[i]=0;
		else
		{
			sort(point_clusters[i].begin(),point_clusters[i].end(),RCS_SortBydist);
			int opt_id=point_clusters[i].size()-1;
			signature[i]=(point_clusters[i][opt_id].dist)/sup_radius;//nomalization
		}
	}
}
void RCS_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points, vector<vector<float>>&Histograms)
{
	int i,j,m;
	//local surface determination via KD-tree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int>pointIdx;
	vector<float>pointDst;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ query_point;

	for(i=0;i<indices.size();i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor(new pcl::PointCloud<pcl::PointXYZ>);//local surface
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_z(new pcl::PointCloud<pcl::PointXYZ>);//subset of the local surface for computing the z-axis of the LRF
		pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_trans(new pcl::PointCloud<pcl::PointXYZ>);//local surface in the LRF system, invariant to rotation
		Vertex x_axis,y_axis,z_axis;
		query_point=cloud->points[indices[i]];
		//
		if(kdtree.radiusSearch(query_point,sup_radius/2,pointIdx,pointDst)>3)//if the point count in the sphere_neighbor_z is smaller than 4, discard computing RCS
		{
			for(j=0;j<pointIdx.size();j++)
			{
				sphere_neighbor_z->points.push_back(cloud->points[pointIdx[j]]);
			}
			LRF_Z_axis(sphere_neighbor_z,z_axis);
		}
		else
		{
			vector<float> RCS_feature_NULL(num_of_rotations*num_of_contour_points,0.0f);
			Histograms.push_back(RCS_feature_NULL);
			continue;
		}
		if(kdtree.radiusSearch(query_point,sup_radius,pointIdx,pointDst)>10)//if the point count in the local surface is smaller than 10, discard computing RCS
		{
			for(j=0;j<pointIdx.size();j++)
			{
				sphere_neighbor->points.push_back(cloud->points[pointIdx[j]]);
			}
			LRF_X_axis(sphere_neighbor,z_axis,sup_radius,pointDst,x_axis);
			LRF_axis(x_axis,z_axis,y_axis);
			LRF pointLRF={indices[i],x_axis,y_axis,z_axis};//LRF calculation 
			transformCloud(sphere_neighbor,pointLRF,sphere_neighbor_trans);//transform the local surface to the LRF system
			vector<float> RCS_feature;
			for(int r=0;r<num_of_rotations;r++)
			{
				float rotate_angle_temp=r*rotate_angle;
				pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_trans_rotated(new pcl::PointCloud<pcl::PointXYZ>);//rotated surface
				RotateCloud(pointLRF,rotate_angle_temp,sphere_neighbor_trans,sphere_neighbor_trans_rotated);
				vector<float> signature;
				RCS_per_rotate(sphere_neighbor_trans_rotated,num_of_contour_points,sup_radius,signature);
				std::copy(signature.begin(),signature.end(),std::back_inserter(RCS_feature));
			}
			Histograms.push_back(RCS_feature);
		}
		else
		{
			vector<float> RCS_feature_NULL(num_of_rotations*num_of_contour_points,0.0f);
			Histograms.push_back(RCS_feature_NULL);
		}
	}
}