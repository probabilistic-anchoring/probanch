#ifndef __ANCHOR_MANAGEMENT_HPP__
#define __ANCHOR_MANAGEMENT_HPP__ 

#include <anchor_msgs/ObjectArray.h> 
#include <anchor_msgs/MovementArray.h>
#include <anchor_msgs/TimedRequest.h> 
#include <anchor_msgs/SpatialRequest.h> 

#include <dc_msgs/AssociationArray.h> 
#include <dc_msgs/MeanPosHiddenArray.h> 

#include <anchoring/anchor_container.hpp>
#include <anchor_utils/ml.hpp>

using namespace std;

// ------------------------------------------
// Class for creating and maintaining anchors
// ------------------------------------------
class AnchorManagement {
  
  // An objects for holding all anchors
  std::unique_ptr<anchoring::AnchorContainer> _anchors;

  // Object classicication 
  ml::MachinePtr _classifier; 
  
  /* --------------
     ROS variables
     -------------- */  
  ros::NodeHandle _nh, _priv_nh;
  ros::Subscriber _object_sub, _track_sub, _assoc_sub;
  ros::ServiceServer _spatial_srv, _timed_srv;
  ros::Publisher _anchor_pub, _display_pub;

  double _time_zero;

  void match( const anchor_msgs::ObjectArrayConstPtr &object_ptr );
  float predict(map< string, map<anchoring::AttributeType, float> > &matches);
  void track( const dc_msgs::MeanPosHiddenArrayConstPtr &movement_ptr );
  void associate( const dc_msgs::AssociationArrayConstPtr &associations_ptr);
  int process( map< string, map<anchoring::AttributeType, float> > &matches, 
	       string &id, 
	       float dist_th,
	       float rate_th = 0.75 );
  bool spatialRequest( anchor_msgs::SpatialRequest::Request &req,
		       anchor_msgs::SpatialRequest::Response &res );
  bool timedRequest( anchor_msgs::TimedRequest::Request &req,
		     anchor_msgs::TimedRequest::Response &res );


public: 
  AnchorManagement(ros::NodeHandle nh);
  ~AnchorManagement() {}
  void init(int threads);
  void spin();
};

#endif // __ANCHOR_MANAGEMENT_HPP__
