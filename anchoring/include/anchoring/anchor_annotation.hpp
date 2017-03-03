#ifndef __ANCHOR_ANNOTATINO_HPP__
#define __ANCHOR_ANNOTATINO_HPP__ 

#include <anchor_msgs/ObjectArray.h> 

#include <anchoring/anchor_container.hpp>

using namespace std;

// ------------------------------------------
// Class for creating and maintaining anchors
// ------------------------------------------
class AnchorAnnotation {
  
  // An object for holding all anchors
  std::unique_ptr<anchoring::AnchorContainer> _anchors;

  // List of attributes of unknown objects
  //std::vector<anchoring::AttributeMap> _objects;
 
  /* --------------
     ROS variables
     -------------- */  
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;
  ros::Subscriber _object_sub;
  
  double _t;
  double _leap_time;
  cv::Mat _img;
  bool _lock_screen;
  bool _processing;

  static const char *window;

  void queue( const anchor_msgs::ObjectArrayConstPtr &object_ptr );

  
  int process( map< string, map<anchoring::AttributeType, float> > &matches, 
	       string &id, 
	       float dist_th,
	       float rate_th = 0.75 );

  // Display functions
  void help();
  static void click_cb(int event, int x, int y, int flags, void *obj);
  void click_wrapper(int event, int x, int y, int flags);

public: 
  AnchorAnnotation(ros::NodeHandle nh);
  ~AnchorAnnotation();

  void init(int threads);
  void spin();
};

#endif // __ANCHOR_MANAGEMENT_HPP__
