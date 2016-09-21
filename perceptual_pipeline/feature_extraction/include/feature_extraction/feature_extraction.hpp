#ifndef __FEATURE_EXTRACTION_HPP__
#define __FEATURE_EXTRACTION_HPP__

#include <image_transport/image_transport.h>

#include <anchor_msgs/ObjectArray.h>

#include <feature_extraction/visual_features.hpp>

class FeatureExtraction {
  
private:

  // ROS node handles 
  ros::NodeHandle nh_; 
  ros::NodeHandle priv_nh_;
  image_transport::ImageTransport it_;

  // Publisher /subscribers
  ros::Subscriber obj_sub_;
  ros::Publisher obj_pub_;
  image_transport::Publisher boxed_pub_;

  // Callbak fn
  void process(const anchor_msgs::ObjectArray::ConstPtr &objects_msg);

public: 
  FeatureExtraction(ros::NodeHandle nh);
  void spin();
  ~FeatureExtraction() {}
};

#endif // __FEATURE_EXTRACTION_HPP__
