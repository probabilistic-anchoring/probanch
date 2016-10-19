#ifndef __OBJECT_TRACKING_HPP__ 
#define __OBJECT_TRACKING_HPP__ 

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

// PCL specific includes
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

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

using namespace pcl::tracking;


class ObjectTracking {

  // Typedefines
  typedef pcl::PointXYZRGBA Point;  
  typedef pcl::PointXYZ SimplePoint;
  typedef ParticleXYZRPY Particle;

  /*
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  typedef typename pcl::search::KdTree<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;
  */

  // Tracker
  std::vector< boost::shared_ptr<ParticleFilterOMPTracker< Point, Particle> > > trackers_;
  std::vector< pcl::PointCloud<Point>::Ptr > references_;

  // Variables
  double factor_;
  double size_;
  int planeMinSize_;
  int clusterMinSize_;
  double angularTh_;
  double distanceTh_;

  

public: 
  ObjectTracking (ros::NodeHandle nh);
  ~ObjectTracking () {}
  void spin();
};

#endif // __OBJECT_TRACKING_HPP__
