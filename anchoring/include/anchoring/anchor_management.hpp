#ifndef __ANCHOR_MANAGEMENT_HPP__
#define __ANCHOR_MANAGEMENT_HPP__ 

/*
  To be moved to a seperate ROS package, and are therefore not added to the repo. yet....
  --------------------------------------------------------------------------------------------
  #include <perceptual_pipeline/ObjectArray.h> 
  #include <perceptual_pipeline/MovementArray.h>
*/

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
  double _time_zero;

  void match( /* const perceptual_pipeline::ObjectArrayConstPtr &object_ptr */ );
  void track( /* const perceptual_pipeline::MovementArrayConstPtr &movement_ptr */ );
  int process( map< string, map<anchoring::AttributeType, float> > &matches, 
	       string &id, 
	       float dist_th,
	       float rate_th = 0.75 );

public: 
  AnchorManagement(ros::NodeHandle nh);
  ~AnchorManagement() {}
  void init(int threads);
  void spin();
};

#endif // __ANCHOR_MANAGEMENT_HPP__
