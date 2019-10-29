/* ------------------------------------------ 
   spatial_features.cpp
   ----------------------

   Class for handling of spatial features such 
   as 3d positions and 3d shapes.
 
   Modified: Oct 29, 2019
   Author: Andreas Persson
   ------------------------------------------ */

#include <pcl/filters/passthrough.h>
#include <features/spatial.hpp>

// ---[ Used namespaces ]--- 
using namespace pcl;
using namespace std;


// ---[ Within namespace...
namespace spatial_3d {

  // Filter the a point cloud based on a pass through filter
  void passThroughFilter( const PointCloud<Point>::Ptr &cloud_ptr,
			  PointCloud<Point>::Ptr &filtered_ptr,
			  const string axis,
			  double min,
			  double max,
			  bool keep_organized ) {
    PointCloud<Point>::Ptr result_ptr (new PointCloud<Point>);
    PassThrough<Point> pass_;
    pass_.setInputCloud (cloud_ptr);
    pass_.setFilterFieldName (axis);
    pass_.setFilterLimits ( min, max);
    pass_.setKeepOrganized (keep_organized);
    pass_.filter (*result_ptr);
    filtered_ptr.swap (result_ptr);
  }

  // Get the 3d postion (only)
  void getPosition( const PointCloud<Point>::Ptr &cloud_ptr,
		    geometry_msgs::Pose &pos ) {
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud_ptr, centroid);

    pos.position.x = centroid[0];
    pos.position.y = centroid[1];
    pos.position.z = centroid[2];

    pos.orientation.x = 0.0;
    pos.orientation.y = 0.0;
    pos.orientation.z = 0.0;
    pos.orientation.w = 1.0;
  }

  // Get 3d postion and orientation
  void getOrientedPosition( const PointCloud<Point>::Ptr &cloud_ptr,
			    geometry_msgs::Pose &pos ) {
    PCA<Point> pca;
    pca.setInputCloud(cloud_ptr);

    // Calculate the position and the orientation
    Eigen::Vector4f centroid = pca.getMean();
    Eigen::Quaternionf rotation = Eigen::Quaternionf(pca.getEigenVectors());

    pos.position.x = centroid[0];
    pos.position.y = centroid[1];
    pos.position.z = centroid[2];

    pos.orientation.x = rotation.x();
    pos.orientation.y = rotation.y();
    pos.orientation.z = rotation.z();
    pos.orientation.w = rotation.w();
  }

  // Get the 3d size (minimal bounding box around a segmented point cloud)
  void getSize( const PointCloud<Point>::Ptr &cloud_ptr,
		geometry_msgs::Vector3 &size ) {
    // Get the 3d bounding box
    Point proj_min;
    Point proj_max;
    getMinMax3D (*cloud_ptr, proj_min, proj_max);

    double width = fabs(proj_max.x - proj_min.x);
    double height = fabs(proj_max.y - proj_min.y);
    double depth = fabs(proj_max.z - proj_min.z);

    size.x = width;
    size.y = height;
    size.z = depth;
  }
  
  
} // ...namespace ('spatial_3d') ]---
