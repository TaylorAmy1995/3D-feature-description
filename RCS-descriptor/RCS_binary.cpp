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
/*********************************************RCS(B-Thresholding: Max-entropy)************************************************/
void RCS_B_thresholding_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,vector<char>&signature)
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
		vector<float> signature_float;
	signature_float.resize(num_of_contour_points);
	for(i=0;i<point_clusters.size();i++)
	{
		if(point_clusters[i].size()==0)
			signature_float[i]=0;
		else
		{
			sort(point_clusters[i].begin(),point_clusters[i].end(),RCS_SortBydist);
			int opt_id=point_clusters[i].size()-1;
			signature_float[i]=(point_clusters[i][opt_id].dist)/sup_radius;//nomalization
		}
	}
	//compute median value
	vector<float> signature_float_copy;
	signature_float_copy=signature_float;
	sort(signature_float_copy.begin(),signature_float_copy.end());
	float Thresh=signature_float_copy[signature_float_copy.size()/2];
	for(i=0;i<signature_float.size();i++)
	{
		if(signature_float[i]>=Thresh)
			signature[i]='1';
		else
			signature[i]='0';
	}

}
void RCS_B_thresholding_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points,vector<vector<char>>&Histograms)
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
			vector<char> RCS_feature_NULL(num_of_rotations*num_of_contour_points,'0');
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
			vector<char> RCS_feature;
			for(int r=0;r<num_of_rotations;r++)
			{
				float rotate_angle_temp=r*rotate_angle;
				pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_trans_rotated(new pcl::PointCloud<pcl::PointXYZ>);//rotated surface
				RotateCloud(pointLRF,rotate_angle_temp,sphere_neighbor_trans,sphere_neighbor_trans_rotated);
				vector<float> signature;
				RCS_B_thresholding_per_rotate(sphere_neighbor_trans_rotated,num_of_contour_points,sup_radius,signature);
				std::copy(signature.begin(),signature.end(),std::back_inserter(RCS_feature));
			}
			Histograms.push_back(RCS_feature);
		}
		else
		{
			vector<char> RCS_feature_NULL(num_of_rotations*num_of_contour_points,'0');
			Histograms.push_back(RCS_feature_NULL);
		}
	}
}
/*********************************************RCS(B-quant)************************************************/
void RCS_B_quant_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,int num_quant_bit,vector<char>&signature)
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
		vector<float> signature_float;
	signature_float.resize(num_of_contour_points);
	for(i=0;i<point_clusters.size();i++)
	{
		if(point_clusters[i].size()==0)
			signature_float[i]=0;
		else
		{
			sort(point_clusters[i].begin(),point_clusters[i].end(),RCS_SortBydist);
			int opt_id=point_clusters[i].size()-1;
			signature_float[i]=(point_clusters[i][opt_id].dist)/sup_radius;//nomalization
		}
	}
	//vector quant
	for(i=0;i<signature_float.size();i++)
	{
		float max_quant_value=pow(2.0f,num_quant_bit)-1;
		vector<char> vec_temp;
		float current_float=signature_float[i];
		for(j=0;j<num_quant_bit;j++)
		{
			float half_max=0.5*pow(2.0f,(num_quant_bit-j))/(max_quant_value+1);
			if(current_float>=half_max) 
			{
				vec_temp.push_back('1');
				current_float-=half_max;
			}
			else vec_temp.push_back('0');
		}
		std::copy(vec_temp.begin(),vec_temp.end(),std::back_inserter(signature));
	}
}
void RCS_B_quant_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int>indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points,int num_quant_bit,vector<vector<char>>&Histograms)
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
			vector<char> RCS_feature_NULL(num_quant_bit*num_of_rotations*num_of_contour_points,'0');
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
			vector<char> RCS_feature;
			for(int r=0;r<num_of_rotations;r++)
			{
				float rotate_angle_temp=r*rotate_angle;
				pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_trans_rotated(new pcl::PointCloud<pcl::PointXYZ>);//rotated surface
				RotateCloud(pointLRF,rotate_angle_temp,sphere_neighbor_trans,sphere_neighbor_trans_rotated);
				vector<float> signature;
				RCS_B_quant_per_rotate(sphere_neighbor_trans_rotated,num_of_contour_points,sup_radius,num_quant_bit,signature);
				std::copy(signature.begin(),signature.end(),std::back_inserter(RCS_feature));
			}
			Histograms.push_back(RCS_feature);
		}
		else
		{
			vector<char> RCS_feature_NULL(num_quant_bit*num_of_rotations*num_of_contour_points,'0');
			Histograms.push_back(RCS_feature_NULL);
		}
	}
}
/*********************************************RCS(B-Geometry)************************************************/
void RCS_B_geometry_per_rotate(PointCloudPtr rotated_cloud,int num_of_contour_points,float sup_radius,vector<char>&signature)
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
		vector<float> signature_float;
	signature_float.resize(num_of_contour_points);
	for(i=0;i<point_clusters.size();i++)
	{
		if(point_clusters[i].size()==0)
			signature_float[i]=0;
		else
		{
			sort(point_clusters[i].begin(),point_clusters[i].end(),RCS_SortBydist);
			int opt_id=point_clusters[i].size()-1;
			signature_float[i]=(point_clusters[i][opt_id].dist)/sup_radius;//nomalization
		}
	}
	//Geometrical encoding
	for(i=0;i<signature_float.size();i++)
	{
		int index_former=i-1,index_current=i;
		if(index_former<0) index_former=signature_float.size()-1;
		if(signature_float[index_current]>=signature_float[index_former]) signature[i]='1';
		else signature[i]='0';
	}

}
void RCS_B_geometry_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<int> indices,float sup_radius,float rotate_angle,int num_of_rotations,
	int num_of_contour_points,vector<vector<char>>&Histograms)
{
	int i,j,m;
	//计算当前搜寻半径下的局部曲面大小
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int>pointIdx;
	vector<float>pointDst;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ query_point;

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
			vector<char> RCS_feature_NULL(num_of_rotations*num_of_contour_points,'0');
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
			vector<char> RCS_feature;
			for(int r=0;r<num_of_rotations;r++)
			{
				float rotate_angle_temp=r*rotate_angle;
				pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_neighbor_trans_rotated(new pcl::PointCloud<pcl::PointXYZ>);//rotated surface
				RotateCloud(pointLRF,rotate_angle_temp,sphere_neighbor_trans,sphere_neighbor_trans_rotated);
				vector<float> signature;
				RCS_B_geometry_per_rotate(sphere_neighbor_trans_rotated,num_of_contour_points,sup_radius,signature);
				std::copy(signature.begin(),signature.end(),std::back_inserter(RCS_feature));
			}
			Histograms.push_back(RCS_feature);
		}
		else
		{
			vector<char> RCS_feature_NULL(num_of_rotations*num_of_contour_points,'0');
			Histograms.push_back(RCS_feature_NULL);
		}
	}
}
