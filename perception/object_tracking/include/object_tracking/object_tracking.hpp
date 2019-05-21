#ifndef __OBJECT_TRACKING_HPP__ 
#define __OBJECT_TRACKING_HPP__ 

#include <iostream>
#include <string>
#include <vector>
#include <map>
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
#include <std_msgs/String.h>

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>



// OpenCV includes
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>

// Support for both opencv2 and opencv3
#if CV_MAJOR_VERSION == 2
#include <opencv2/contrib/contrib.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/video/background_segm.hpp>
#endif


// Anchoring includes
#include <anchor_msgs/ClusterArray.h>
// ---------------------------------

using namespace std;
using namespace pcl::tracking;

// ---------------------------------

class ObjectTracking {

  // Typedefines
  typedef pcl::PointXYZRGBA Point;  
  typedef pcl::PointXYZ SimplePoint;
  typedef ParticleXYZRPY Particle;
  typedef ParticleFilterTracker<Point, Particle> ParticleFilter;
  enum {
    MIN_ACTIVITY = 0,
    MAX_ACTIVITY = 100,
    DECREASE_ACTIVITY = -5
  };
  
  // Tracker struct
  struct Tracker {

    // Tracker variables
    boost::shared_ptr<ParticleFilter> tracker_;
    int activity_;
    Eigen::Vector4f center_;

    // Constructor
    Tracker( const pcl::PointCloud<Point>::Ptr &cluster_ptr, 
	     Eigen::Vector4f &center,
	     bool use_fixed = true );
    void setActivity(int activity);
    float getActivity();
    bool isActive();
  };
  vector<Tracker> trackers_;
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

  // Display 
  bool display_image_;
  ros::Subscriber display_trigger_sub_;
  image_transport::Publisher display_image_pub_;
  cv::Mat result_img_;

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
  void triggerCb( const std_msgs::String::ConstPtr &msg);
  void trackCb( const sensor_msgs::Image::ConstPtr &rgb_msg, 
		      const sensor_msgs::Image::ConstPtr &depth_msg, 
		      const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg, 
		      const sensor_msgs::PointCloud2::ConstPtr &cloud_msg );
  void clustersCb ( const anchor_msgs::ClusterArray::ConstPtr &msg);
  void activate ( const pcl::PointCloud<Point>::Ptr &pts);
  void process (const pcl::PointCloud<Point>::Ptr &cloud_ptr);
  float distance ( const Point &pt, const Eigen::Vector4f &center);
  void drawParticles ( cv::Mat &img, const image_geometry::PinholeCameraModel &model);
  void gridSampleApprox ( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
			  pcl::PointCloud<Point>::Ptr &result_ptr, 
			  double leaf_size = 0.002 );
  void radiusSearch ( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
		      pcl::PointCloud<Point>::Ptr &result_ptr, 
		      double radius = 0.25 );

public: 
  ObjectTracking (ros::NodeHandle nh, bool useApprox = true);
  ~ObjectTracking ();
  void spin();
};

#endif // __OBJECT_TRACKING_HPP__
