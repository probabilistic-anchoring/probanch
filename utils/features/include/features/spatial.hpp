#ifndef __SPATIAL_HPP__
#define __SPATIAL_HPP__

#include <iostream>
#include <vector>

// PCL common includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>

// ROS includes
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

namespace spatial_3d {

  // ---[ Used namespaces ]---
  using namespace std;

  // ---[ Type defines ]---
  typedef pcl::PointXYZRGBA Point;  // pcl::PointXYZ OR pcl::PointXYZRGB OR pcl::PointXYZRGBA
  typedef pcl::PointXYZ SimplePoint;
  
  // -------------------
  // Namespace functions
  // -------------------
  void passThroughFilter( const pcl::PointCloud<SimplePoint>::Ptr &cloud_ptr,
				       pcl::PointCloud<SimplePoint>::Ptr &filtered_ptr,
				       const string axis,
				       double min,
				       double max,
				       bool keep_organized = true );
  void getPosition( const pcl::PointCloud<SimplePoint>::Ptr &cloud_ptr,
		    geometry_msgs::Pose &pos );
  void getOrientedPosition( const pcl::PointCloud<SimplePoint>::Ptr &cloud_ptr,
			    geometry_msgs::Pose &pos );
  void getSize( const pcl::PointCloud<SimplePoint>::Ptr &projected_cloud_ptr,
		geometry_msgs::Vector3 &size );
  
} // namespace 'spatial_3d'

#endif // __VISUAL_HPP__
