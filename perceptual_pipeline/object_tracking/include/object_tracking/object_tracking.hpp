#ifndef __OBJECT_TRACKING_HPP__ 
#define __OBJECT_TRACKING_HPP__ 

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <tf/transform_listener.h>

// PCL includes
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/search/pcl_search.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <boost/foreach.hpp>

// Anchoring includes
#include <anchor_msgs/ClusterArray.h>

// Macros
#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
}
// ---------------------------------

using namespace std;
using namespace pcl::tracking;

class ObjectTracking {

  // Typedefines
  typedef pcl::PointXYZRGBA Point;  
  typedef pcl::PointXYZ SimplePoint;
  typedef ParticleXYZRPY Particle;

  // Tracker variables
  vector< boost::shared_ptr<ParticleFilterOMPTracker< Point, Particle> > > trackers_;
  vector< pcl::PointCloud<Point>::Ptr > references_;

  // Variables
  double factor_;
  double size_;
  int planeMinSize_;
  int clusterMinSize_;
  double angularTh_;
  double distanceTh_;

  boost::mutex mtx_;

  // Node handler(s) 
  ros::NodeHandle nh_, priv_nh_;
  //image_transport::ImageTransport it_;

  // Tf transform listener 
  tf::TransformListener *tf_listener_;
  string base_frame_;
  
  // Private functions
  //void trackCallback( const sensor_msgs::Image::ConstPtr image, const sensor_msgs::CameraInfo::ConstPtr camera_info, const sensor_msgs::PointCloud2::ConstPtr cloud);
  void clustersCallback ( const anchor_msgs::ClusterArray::ConstPtr &msg);
  void init ( const pcl::PointCloud<Point>::Ptr &pts);
  float distance ( const Point &pt, const Eigen::Vector4f &center);
  void drawParticles ();

public: 
  ObjectTracking (ros::NodeHandle nh);
  ~ObjectTracking () {}
  void spin();
};

#endif // __OBJECT_TRACKING_HPP__
