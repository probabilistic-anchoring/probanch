#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <anchor_msgs/DisplayArray.h>
#include <anchor_msgs/AnchorArray.h>

#include <anchoring/anchor_management.hpp>

using namespace cv;
using namespace std;
using namespace anchoring;
  
// Constructor
AnchorManagement::AnchorManagement(ros::NodeHandle nh) : _nh(nh), _priv_nh("~") {
  
  _object_sub = _nh.subscribe("/objects/classified", 10, &AnchorManagement::match, this);
  //_track_sub = _nh.subscribe("/movements", 10, &AnchorManagement::track, this);
  _track_sub = _nh.subscribe("/associations", 10, &AnchorManagement::track, this);

  _timed_srv = _nh.advertiseService("/anchoring/timed_request", &AnchorManagement::timedRequest, this);
  _spatial_srv = _nh.advertiseService("/anchoring/spatial_request", &AnchorManagement::spatialRequest, this);

  // Publisher used for collecting bag files
  _anchor_pub = _nh.advertise<anchor_msgs::AnchorArray>("/anchors", 1);
  
  // Publischer used for displaying the anchors
  _display_pub = _nh.advertise<anchor_msgs::DisplayArray>("/display/anchors", 1);

  // Create the anchor map
  std::string db;
  if( _priv_nh.getParam("db_name", db) ) {
    ROS_WARN("Using database : %s", db.c_str());
    _anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", db) );
  }
  else {  // ...default databas
    ROS_WARN("Using default database (anchordb).");
    _anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", "anchordb") );
  }

  // Load train classifier
  _classifier = ml::load( db, "ml", "svm");
  
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
  
  ROS_WARN("Got message!");
  // Get the time
  ros::Time t = object_ptr->header.stamp;
  if( _time_zero < 0.0 ) {
    _time_zero = t.toSec();
  }
  
  // Maintain all incoming objectss
  for( uint i = 0; i < object_ptr->objects.size(); i++) {
    /*
    // Read percept from ROS message
    cv_bridge::CvImagePtr cv_ptr;
    Mat img, descriptor, histogram;
    try {
      cv_ptr = cv_bridge::toCvCopy( object_ptr->objects[i].descriptor.data, 
				    sensor_msgs::image_encodings::MONO8 );
      cv_ptr->image.copyTo(descriptor);
      cv_ptr = cv_bridge::toCvCopy( object_ptr->objects[i].caffe.data,
				    sensor_msgs::image_encodings::BGR8 );
      cv_ptr->image.copyTo(img);

      cv_ptr = cv_bridge::toCvCopy( object_ptr->objects[i].color.data,
				    sensor_msgs::image_encodings::TYPE_32FC1 );
      cv_ptr->image.copyTo(histogram);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("[AnchorManagement::match] receiving descriptor or image: %s", e.what());
      return;
    }
    */

    // Create a map of all object attributes
    AttributeMap attributes;
    try {
      attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(object_ptr->objects[i].descriptor) );
      attributes[COLOR] = AttributePtr( new ColorAttribute(object_ptr->objects[i].color) );
      attributes[SHAPE] = AttributePtr( new ShapeAttribute(object_ptr->objects[i].shape) );
      attributes[POSITION] = AttributePtr( new PositionAttribute(object_ptr->objects[i].position) );    
      attributes[CAFFE] = AttributePtr( new CaffeAttribute(object_ptr->objects[i].caffe) ); 
    } catch( const std::exception &e ) {
      ROS_ERROR("[AnchorManagement::match]%s", e.what() );
      continue;
    }

    // Match all attributes
    map< string, map<anchoring::AttributeType, float> > matches;
    this->_anchors->match( attributes, matches);

    // Process the matches
    std::string id; 
    for( auto ite = matches.begin(); ite != matches.end(); ++ite) {
      cv::Mat sample( 1, 5, CV_32F);
      sample.at<float>( 0, 0) = ite->second[CAFFE];
      sample.at<float>( 0, 1) = ite->second[COLOR];
      sample.at<float>( 0, 2) = ite->second[POSITION];
      sample.at<float>( 0, 3) = ite->second[SHAPE];
      sample.at<float>( 0, 4) = 2.0 / (1.0 + exp( abs(this->_anchors->diff( ite->first, t)) ));

      // Classify the sample
      float pred = this->_classifier->predict(sample);
      if( pred > 0.9 ) {
	id = ite->first;
	break;
      }
    }
    if( !id.empty() ) {
      this->_anchors->re_acquire(id, attributes, t ); // RE_ACQUIRE
    }
    else {
      this->_anchors->acquire(attributes, t); // ACQUIRE

    }
    /*
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
    */
  }

  
  // Get a snapshot of all anchors seen in the scene at time t
  anchor_msgs::AnchorArray msg;
  this->_anchors->getArray<anchor_msgs::Anchor>( msg.anchors, t );
  this->_anchor_pub.publish(msg);
  
  // Get all information about anchors (for display purposes )
  anchor_msgs::DisplayArray display_msg;
  display_msg.header = object_ptr->header;
  display_msg.image = object_ptr->image;
  display_msg.info = object_ptr->info;
  display_msg.transform = object_ptr->transform;
  this->_anchors->getArray<anchor_msgs::Display>( display_msg.anchors, t );
  this->_display_pub.publish(display_msg);

  ROS_INFO("Anchors: %d", (int)this->_anchors->size());
  //ROS_INFO("Time: %.2f", t.toSec() - _time_zero);
  ROS_INFO("------------------------------");
}

/* -----------------------------------------
   Main track function(s)
   --------------------------------------- */
/*
void AnchorManagement::track( const anchor_msgs::MovementArrayConstPtr &movement_ptr ) {
  
  // Maintain all incoming object movmements
  for( uint i = 0; i < movement_ptr->movements.size(); i++) {
    anchoring::AttributeMap attributes;
    attributes[POSITION] = AttributePtr( new PositionAttribute(movement_ptr->movements[i]) );    

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
*/

// ---[ Track method based on data association ]---
void AnchorManagement::track( const dc_msgs::AssociationArrayConstPtr &associations_ptr ) {
  
  ROS_INFO("Got associations.");

  try {
  
    // Update (merge) anchors based on probabilistic object tracking
    for( auto &msg: associations_ptr->associations) {
      int idx_id = -1;
      int idx_best = -1;
      float best = 0.0;
      std::cout << "Id: " << this->_anchors->toString(msg.id) << std::endl;
      for( uint i = 0; i < msg.associations.size(); i++) {
	std::cout << "Assoc: " << this->_anchors->toString(msg.associations[i]);
	std::cout << " (" << msg.probabilities[i] << ")" << std::endl;
	if( msg.probabilities[i] > best ) {
	  best = msg.probabilities[i]; 
	  idx_best = i;
	}
	if(msg.id == msg.associations[i]){
	  idx_id = i;
	}
      }
    
      //if( msg.associations[idx] != msg.id && best > 0.5 ) {
      if (idx_id==-1){
	if( best > 0.80 ) {
	  this->_anchors->track( msg.associations[idx_best], msg.id); // TRACK
	}
      }

      else {
	if( msg.associations[idx_best] != msg.id && msg.probabilities[idx_id] < associations_ptr->gamma_newT && best > 0.80 ) {
	  this->_anchors->track( msg.associations[idx_best], msg.id); // TRACK
	}
	else if (msg.associations[idx_best] == msg.id && best > 0.80){
	  continue;
	}
      }    
    }
  }
  catch( const std::exception &e ) {
    ROS_ERROR("[AnchorManagement::track]%s", e.what() );
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
    //std::cout << "Prob. dist: " << ite->second[POSITION] << ", prob. rate: " << ite->second[IMAGE] << std::endl;
    if( ite->second[POSITION] > best ) {
      best = ite->second[POSITION];
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
bool AnchorManagement::timedRequest( anchor_msgs::TimedRequest::Request &req,
				     anchor_msgs::TimedRequest::Response &res ) {

  // Get a snapshot of all anchors seen scene at time t
  this->_anchors->getArray<anchor_msgs::Anchor>( res.anchors, req.t );
  return true;
}

// Handle a request for spatial information about one (or more) anchor(s)
bool AnchorManagement::spatialRequest( anchor_msgs::SpatialRequest::Request &req,
				       anchor_msgs::SpatialRequest::Response &res ) {
  try {

    // Iterate given ids and collect spatial positions and volumes
    for( auto &id : req.ids) {
      res.anchors.push_back(this->_anchors->get<anchor_msgs::Anchor>(id));
    }
  }
  catch( const std::exception &e ) {
    ROS_ERROR("[AnchorManagement::spatialRequest]%s", e.what() );
    return false;
  }
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
