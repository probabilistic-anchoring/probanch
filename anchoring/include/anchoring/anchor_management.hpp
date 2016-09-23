#ifndef __ANCHOR_MANAGEMENT_HPP__
#define __ANCHOR_MANAGEMENT_HPP__ 

#include <anchor_msgs/ObjectArray.h> 
#include <anchor_msgs/MovementArray.h>
#include <anchor_msgs/AnchorRequest.h> 

#include <anchoring/anchor_container.hpp>

using namespace std;

// ------------------------------------------
// Class for creating and maintaining anchors
// ------------------------------------------
class AnchorManagement {
  
  // An objects for holding all anchors
  std::unique_ptr<anchoring::AnchorContainer> _anchors;
 
  /* --------------
     ROS variables
     -------------- */  
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;
  ros::Subscriber _object_sub, _track_sub;
  ros::ServiceServer _anchor_srv;

  double _time_zero;

  void match( const anchor_msgs::ObjectArrayConstPtr &object_ptr );
  void track( const anchor_msgs::MovementArrayConstPtr &movement_ptr );
  int process( map< string, map<anchoring::AttributeType, float> > &matches, 
	       string &id, 
	       float dist_th,
	       float rate_th = 0.75 );
  bool request( anchor_msgs::AnchorRequest::Request &req,
		anchor_msgs::AnchorRequest::Response &res );


public: 
  AnchorManagement(ros::NodeHandle nh);
  ~AnchorManagement() {}
  void init(int threads);
  void spin();
};

#endif // __ANCHOR_MANAGEMENT_HPP__
