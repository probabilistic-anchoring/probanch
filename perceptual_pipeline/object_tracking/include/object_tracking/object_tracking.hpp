#ifndef __OBJECT_TRACKING_HPP__ 
#define __OBJECT_TRACKING_HPP__ 

#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

#include <boost/foreach.hpp>

// ROS includes
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// PCL includes
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>

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

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp> 


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
  typedef ParticleFilterTracker<Point, Particle> ParticleFilter;

  // Tracker variables
  vector< boost::shared_ptr<ParticleFilterOMPTracker< Point, Particle> > > trackers_;
  //vector< boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker< Point, Particle> > > trackers_;
  vector< pcl::PointCloud<Point>::Ptr > references_;
  vector< Eigen::Vector4f > centers_;

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
  image_transport::ImageTransport it_;

  // Publisher /subscribers
  const size_t queueSize_;
  image_transport::SubscriberFilter *rgb_sub_;
  image_transport::SubscriberFilter *depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
  ros::Subscriber clusters_sub_;

  // Defined sync policies
  bool useApprox_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ApproximateSyncPolicy;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact_;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate_;

  // Tf transform listener 
  tf::TransformListener *tf_listener_;
  string base_frame_;

  // Background subtraction
  cv::Mat past_, prev_;
  cv::Ptr<cv::BackgroundSubtractor> depth_subtractor_;
  cv::Ptr<cv::BackgroundSubtractor> rgb_subtractor_;
  
  // Private functions
  void trackCallback( const sensor_msgs::Image::ConstPtr &rgb_msg, 
		      const sensor_msgs::Image::ConstPtr &depth_msg, 
		      const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg, 
		      const sensor_msgs::PointCloud2::ConstPtr &cloud_msg );
  void clustersCallback ( const anchor_msgs::ClusterArray::ConstPtr &msg);
  void init ( const pcl::PointCloud<Point>::Ptr &pts);
  void process (const pcl::PointCloud<Point>::Ptr &cloud_ptr);
  float distance ( const Point &pt, const Eigen::Vector4f &center);
  void drawParticles ( cv::Mat &img, const image_geometry::PinholeCameraModel &model);
  void removeZeroPoints ( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			  pcl::PointCloud<Point>::Ptr &result_ptr );

public: 
  ObjectTracking (ros::NodeHandle nh, bool useApprox = true);
  ~ObjectTracking ();
  void spin();
};

#endif // __OBJECT_TRACKING_HPP__
