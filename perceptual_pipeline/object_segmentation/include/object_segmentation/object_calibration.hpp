#ifndef __OBJECT_CALIBRATION_HPP__ 
#define __OBJECT_CALIBRATION_HPP__ 

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

class ObjectCalibration {

  // Defined sync policies
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
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate_;

  // Display functions
  static const char* window_;
  cv::Mat display_img_;
  int plane_size_, cluster_size_, angle_th_, distance_th_, factor_;

  // Private functions
  void segmentationCb( const sensor_msgs::Image::ConstPtr image, const sensor_msgs::CameraInfo::ConstPtr camera_info, const sensor_msgs::PointCloud2::ConstPtr cloud);
  void filter( pcl::PointCloud<segmentation::Point>::Ptr &cloud_ptr );

  // Segmentation 
  segmentation::Segmentation seg_;

public: 
  ObjectCalibration(ros::NodeHandle nh);
  ~ObjectCalibration();
  void spin();
};

#endif // __OBJECT_CALIBRATION_HPP__
