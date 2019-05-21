#ifndef __FEATURE_EXTRACTION_HPP__
#define __FEATURE_EXTRACTION_HPP__

#include <image_transport/image_transport.h>

#include <std_msgs/String.h>
#include <anchor_msgs/ObjectArray.h>

#include <features/visual.hpp>

class FeatureExtraction {
  
private:

  // Color feature object 
  visual_2d::ColorFeatures _cf; 

  // ROS node handles 
  ros::NodeHandle nh_; 
  ros::NodeHandle priv_nh_;
  image_transport::ImageTransport it_;

  // Publisher /subscribers
  ros::Subscriber obj_sub_;
  ros::Publisher obj_pub_;
  //image_transport::Publisher boxed_pub_;

  std::string display_image_;
  ros::Subscriber display_trigger_sub_;
  image_transport::Publisher display_image_pub_;

  // Callback functions
  void triggerCb( const std_msgs::String::ConstPtr &msg);
  void processCb(const anchor_msgs::ObjectArray::ConstPtr &objects_msg);

public: 
  FeatureExtraction(ros::NodeHandle nh);
  void spin();
  ~FeatureExtraction() {}
};

#endif // __FEATURE_EXTRACTION_HPP__
