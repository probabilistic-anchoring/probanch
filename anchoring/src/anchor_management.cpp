#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <anchor_msgs/DisplayArray.h>
#include <anchor_msgs/SnapshotArray.h>

#include <anchoring/anchor_management.hpp>

using namespace cv;
using namespace std;
using namespace anchoring;
  
// Constructor
AnchorManagement::AnchorManagement(ros::NodeHandle nh) : _nh(nh), _priv_nh("~") {
  
  _object_sub = _nh.subscribe("/objects/classified", 10, &AnchorManagement::match, this);
  _track_sub = _nh.subscribe("/movements", 10, &AnchorManagement::track, this);

  _anchor_srv = _nh.advertiseService("anchor_request", &AnchorManagement::request, this);

  // Publisher used for collecting bag files
  _anchor_pub = _nh.advertise<anchor_msgs::SnapshotArray>("/anchors/snapshot", 1);
  
  // Publischer used for displaying the anchors
  _display_pub = _nh.advertise<anchor_msgs::DisplayArray>("/display/anchors", 1);

  // Create the anchor map
  //_anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", "anchorspace") );
  _anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", "testspace") );
  _time_zero = -1.0;
}

//  Init function
void AnchorManagement::init(int threads) {
  ROS_WARN("Start loading...");
  this->_anchors->init(threads);
  ROS_WARN("    ...[done]");
}

/* -----------------------------------------
   Main matcher function 
   ----------------------
   Receives and process each incomming 
   scene of object(s).
   --------------------------------------- */
void AnchorManagement::match( const anchor_msgs::ObjectArrayConstPtr &object_ptr ) {
  
  // Get the time
  ros::Time t = object_ptr->header.stamp;
  if( _time_zero < 0.0 ) {
    _time_zero = t.toSec();
  }
  
  // Maintain all incoming objectss
  for( uint i = 0; i < object_ptr->objects.size(); i++) {

    // Read percept from ROS message
    cv_bridge::CvImagePtr cv_ptr;
    Mat img, descriptor, histogram;
    try {
      cv_ptr = cv_bridge::toCvCopy( object_ptr->objects[i].descriptor.data, 
				    sensor_msgs::image_encodings::MONO8 );
      cv_ptr->image.copyTo(descriptor);
      ROS_WARN("Fine #1!");
      cv_ptr = cv_bridge::toCvCopy( object_ptr->objects[i].caffe.data,
				    sensor_msgs::image_encodings::BGR8 );
      cv_ptr->image.copyTo(img); 
      ROS_WARN("Fine #2!");
      cv_ptr = cv_bridge::toCvCopy( object_ptr->objects[i].color.data,
				    sensor_msgs::image_encodings::TYPE_32FC1 );
      cv_ptr->image.copyTo(histogram); 
      ROS_WARN("Fine #3!");
      
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("[AnchorManagement::match] receiving descriptor or image: %s", e.what());
      return;
    }

    // Create a map of all object attributes
    AttributeMap attributes;
    attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(descriptor) );
    ROS_WARN("[Descriptor] fine!");
    attributes[COLOR] = AttributePtr( new ColorAttribute( histogram, object_ptr->objects[i].caffe.predictions, object_ptr->objects[i].color.symbols) );
    ROS_WARN("[Color] fine!");
    attributes[SHAPE] = AttributePtr( new ShapeAttribute( object_ptr->objects[i].shape.data, object_ptr->objects[i].shape.symbols) );
    attributes[LOCATION] = AttributePtr( new LocationAttribute( object_ptr->objects[i].location.data, object_ptr->objects[i].location.symbols) );    
    attributes[CAFFE] = AttributePtr( new CaffeAttribute(img, object_ptr->objects[i].caffe.border, object_ptr->objects[i].caffe.predictions, object_ptr->objects[i].caffe.symbols) ); 
    
    // Match all attributes
    map< string, map<anchoring::AttributeType, float> > matches;
    this->_anchors->match( attributes, matches);

    // Process matches
    string id;
    int result = this->process( matches, id, 0.95, 0.65);
    if( result != 0 ) {
      this->_anchors->re_acquire(id, attributes, t, (result > 0 ? true : false) ); // RE_ACQUIRE
    }
    else {
      this->_anchors->acquire(attributes, t); // ACQUIRE
    } 
  }

  
  // Get a snapshot of all anchors seen scene at time t
  anchor_msgs::SnapshotArray snap_msg;
  this->_anchors->getArray<anchor_msgs::Snapshot>( snap_msg.anchors, t );
  this->_anchor_pub.publish(snap_msg);
  
  // Get all infromation about anchors (for display purposes )
  anchor_msgs::DisplayArray display_msg;
  display_msg.header = object_ptr->header;
  display_msg.image = object_ptr->image;
  this->_anchors->getArray<anchor_msgs::Display>( display_msg.anchors, t );
  this->_display_pub.publish(display_msg);

  ROS_INFO("Anchors: %d", (int)this->_anchors->size());
  //ROS_INFO("Time: %.2f", t.toSec() - _time_zero);
  ROS_INFO("------------------------------");
}

/* -----------------------------------------
   Main track function
   --------------------------------------- */
void AnchorManagement::track( const anchor_msgs::MovementArrayConstPtr &movement_ptr ) {
  
  // Maintain all incoming object movmements
  for( uint i = 0; i < movement_ptr->movements.size(); i++) {
    anchoring::AttributeMap attributes;
    attributes[LOCATION] = AttributePtr( new LocationAttribute(movement_ptr->movements[i]) );    

    // Match the location attribute
    map< string, map<AttributeType, float> > matches;
    this->_anchors->match( attributes, matches);

    // Process matches
    string id;
    ros::Time t = movement_ptr->movements[i].header.stamp;
    if( this->process( matches, id, 0.25) != 0 ) {
      this->_anchors->track(id, attributes, t); // TRACK
    }
    else {      
      this->_anchors->acquire(attributes, t); // ACQUIRE
    }
  } 
}

/* -----------------------------------------
   Process function  
   -----------------
   Process all matches and return true in 
   case an object is re-observed.
   ---
   All matching values are in range 
   [0.0 1.0] (where 1.0 is a perfect match).
   ---

   Return:
     -1 - tracked by distance
      0 - no match
     +1 - re-acquried by match
   -------------------------------------- */
int AnchorManagement::process( map< string, map<AttributeType, float> > &matches, 
			       string &id, 
			       float dist_th,
			       float rate_th ) {
 
  // Check the location (main feature for track)
  float best = 0.0;
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {
    //std::cout << "Prob. dist: " << ite->second[LOCATION] << ", prob. rate: " << ite->second[IMAGE] << std::endl;
    if( ite->second[LOCATION] > best ) {
      best = ite->second[LOCATION];
      id = ite->first;
    }
  }
  if( best > dist_th ) {
    return -1;
  }

  // Check keypoints, caffe-result, color and shape (main feature for acquire/re_acquire)
  best = 0.0;
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {

    // ( DESCRIPTOR OR CAFFE ) AND ( COLOR AND SHAPE )
    float rate_1 = max( ite->second[DESCRIPTOR], ite->second[CAFFE] ); 
    float rate_2 = min( ite->second[COLOR], ite->second[SHAPE] );      
    float rate = min( rate_1, rate_2 ); 
    if( rate > best ) {
      best = rate;      
      id = ite->first;
    }
  }
  if( best > rate_th ) {
    return 1;
  }

  return 0;
}

// Handle a request for a snapshot the the anchorspace
bool AnchorManagement::request( anchor_msgs::AnchorRequest::Request &req,
				anchor_msgs::AnchorRequest::Response &res ) {

  // Get a snapshot of all anchors seen scene at time t
  this->_anchors->getArray<anchor_msgs::Snapshot>( res.anchors, req.t );
  return true;
}


// For 'ROS loop' access
void AnchorManagement::spin() {
  ros::Rate rate(30);
  while (ros::ok()) {

    // Maintain the anchor list while idle
    if( !this->_anchors->empty() ) {
      this->_anchors->maintain();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

// ------------------------------------------
// Main function
// ------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "anchoring_management");
  ros::NodeHandle nh;

  // Reading the number of threads used to initilize the anchor space with
  int threads;
  if( !ros::param::get("~threads", threads) ) {
    threads = 8;  // Defualt: 8
  }

  AnchorManagement node(nh);
  node.init(threads);

  node.spin();
  return 0;
}
