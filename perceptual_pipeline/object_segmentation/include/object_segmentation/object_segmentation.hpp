#ifndef __OBJECT_SEGMENTATION_HPP__ 
#define __OBJECT_SEGMENTATION_HPP__ 

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <object_segmentation/pcl_segmentation.hpp>

class ObjectSegmentation {

  // Defined sync policies
  bool useApprox_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ApproximateSyncPolicy;

  // Node handler(s) 
  ros::NodeHandle nh_, priv_nh_;
  image_transport::ImageTransport it_;

  // Tf transform listener 
  tf::TransformListener *tf_listener_;
  std::string base_frame_;

  // Spatial thresholds
  double min_x_, max_x_;
  double min_y_, max_y_;
  double min_z_, max_z_;

  // Subscribers / publishers
  const size_t queueSize_;
  image_transport::SubscriberFilter *image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
  ros::Publisher obj_pub_;
  ros::Publisher cluster_pub_;
  ros::Publisher move_pub_;
  ros::ServiceClient _tracking_client;
  
  message_filters::Synchronizer<ExactSyncPolicy> *syncExact_;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate_;

  // Display 
  bool display_image_, display_window_;
  ros::Subscriber display_trigger_sub_;
  image_transport::Publisher display_image_pub_;
  cv::Mat result_img_;
  
  // Private functions
  void triggerCb( const std_msgs::String::ConstPtr &msg);
  void segmentationCb( const sensor_msgs::Image::ConstPtr image, const sensor_msgs::CameraInfo::ConstPtr camera_info, const sensor_msgs::PointCloud2::ConstPtr cloud);
  void filter( pcl::PointCloud<segmentation::Point>::Ptr &cloud_ptr );
  std::vector<cv::Point> contoursConvexHull( std::vector<std::vector<cv::Point> > contours );

  
  // Segmentation 
  segmentation::Segmentation seg_;

public: 
  ObjectSegmentation(ros::NodeHandle nh, bool useApprox = true);
  ~ObjectSegmentation();
  void spin();
};

#endif // __OBJECT_SEGMENTATION_HPP__
