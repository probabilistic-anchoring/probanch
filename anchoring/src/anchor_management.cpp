#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <anchoring/anchor_management.hpp>

using namespace cv;
using namespace std;
using namespace anchoring;
  
// Constructor
AnchorManagement::AnchorManagement(ros::NodeHandle nh) : _nh(nh), _priv_nh("~") {
  
  _object_sub = _nh.subscribe("/objects", 10, &AnchorManagement::match, this);
  _track_sub = _nh.subscribe("/movements", 10, &AnchorManagement::track, this);

  // Create the anchor map
  _anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", "anchorspace") );
  _time_zero = -1.0;
}

//  Init function
void AnchorManagement::init(int threads) {
  ROS_WARN("Start loading...");
  this->_anchors->init(threads);
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
  for( uint i = 0; i < object_ptr->caffes.size(); i++) {

    // Read percept from ROS message
    cv_bridge::CvImagePtr cv_ptr;
    Mat img, descriptor;
    try {
      cv_ptr = cv_bridge::toCvCopy( object_ptr->descriptors[i].data, 
				    sensor_msgs::image_encodings::MONO8 );
      cv_ptr->image.copyTo(descriptor);
      cv_ptr = cv_bridge::toCvCopy( object_ptr->caffes[i].data,
				    sensor_msgs::image_encodings::BGR8 );
      cv_ptr->image.copyTo(img); 
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("[AnchorManagement::match] receiving descriptor or image: %s", e.what());
    }

    // Create a map of all object attributes
    AttributeMap attributes;
    attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(descriptor) );
    attributes[COLOR] = AttributePtr( new ColorAttribute( object_ptr->colors[i].data, object_ptr->colors[i].symbols) );
    attributes[SHAPE] = AttributePtr( new ShapeAttribute( object_ptr->shapes[i].data, object_ptr->shapes[i].symbols) );
    attributes[LOCATION] = AttributePtr( new LocationAttribute( object_ptr->locations[i].data, object_ptr->locations[i].symbols) );    
    attributes[CAFFE] = AttributePtr( new CaffeAttribute(img, object_ptr->caffes[i].predictions, object_ptr->caffes[i].symbols) ); 
    
    // Match all attributes
    map< string, map<anchoring::AttributeType, float> > matches;
    this->_anchors->match( attributes, matches);

    // Process matches
    string id;
    int result = this->process( matches, id, 0.05, 0.65);
    if( result != 0 ) {
      this->_anchors->re_acquire(id, attributes, t, (result > 0 ? true : false) ); // RE_ACQUIRE
    }
    else {
      this->_anchors->acquire(attributes, t); // ACQUIRE
    } 
  }
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
  float best = dist_th + 0.00001;  // <-- Add some momentum 
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {
    if( ite->second[LOCATION] < best ) {
      best = ite->second[LOCATION];
      id = ite->first;
    }
  }
  if( best < dist_th ) {
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
