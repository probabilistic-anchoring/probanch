#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <anchor_msgs/DisplayArray.h>
#include <anchor_msgs/AnchorArray.h>
#include <anchor_msgs/RemovedAnchorArray.h>

#include <anchoring/anchor_management.hpp>

using namespace cv;
using namespace std;
using namespace anchoring;

// Constructor
AnchorManagement::AnchorManagement(ros::NodeHandle nh) : _nh(nh), _priv_nh("~") {

  _object_sub = _nh.subscribe("/objects/classified", 10, &AnchorManagement::match, this);
  _track_sub = _nh.subscribe("/logic_anchors", 10, &AnchorManagement::track, this);
  _update_sub = _nh.subscribe("/semantic_update", 10, &AnchorManagement::update, this);
  //_assoc_sub = _nh.subscribe("/associations", 10, &AnchorManagement::associate, this);

  _timed_srv = _nh.advertiseService("/anchoring/timed_request", &AnchorManagement::timedRequest, this);
  _spatial_srv = _nh.advertiseService("/anchoring/spatial_request", &AnchorManagement::spatialRequest, this);

  // Publisher used for further broadcasting anchor information
  _anchor_pub = _nh.advertise<anchor_msgs::AnchorArray>("/anchors", 1);
  _remove_pub = _nh.advertise<anchor_msgs::RemovedAnchorArray>("/anchors_removed", 1);
  
  // Publischer used for displaying the anchors
  _display_pub = _nh.advertise<anchor_msgs::DisplayArray>("/display/anchors", 1);

  // Create the anchor map
  std::string db;
  if( _priv_nh.getParam("db_name", db) ) {
    if( db == "none" ) {
      ROS_WARN("Using no database, anchors ponly stored in working memory.");
      _anchors = std::unique_ptr<AnchorCollection>( new AnchorCollection() );
    }
    else {
      ROS_WARN("Using database : %s", db.c_str());
      _anchors = std::unique_ptr<AnchorCollection>( new AnchorCollection("anchors", db) );
    }
  }
  else {  // ...default databas
    ROS_WARN("Using default database (anchordb).");
    _anchors = std::unique_ptr<AnchorCollection>( new AnchorCollection("anchors", "anchordb") );
  }

  // Load train classifier
  //_classifier = ml::load( db, "ml", "svm");
  const string path = ros::package::getPath("anchoring");
  _classifier = machine::load( path + "/models/svm.yml", "svm");

  _time_zero = -1.0;

  auto mongo_instance = mongo::Database::create_instance();
}

//  Init function
void AnchorManagement::init(int threads) {
  if( this->_anchors->persistent() ) {
    ROS_WARN("Start loading...");
    this->_anchors->init(threads);
    ROS_WARN("    ...[done]");
  }
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

  // Maintain all incoming objects
  std::vector<ObjectMatching> objects;
  for( uint i = 0; i < object_ptr->objects.size(); i++) {

    // Create a map of all object attributes
    AttributeMap attributes;
    try {
      attributes[CATEGORY] = AttributePtr( new CategoryAttribute(object_ptr->objects[i].category) );
      attributes[COLOR] = AttributePtr( new ColorAttribute(object_ptr->objects[i].color) );
      if( !object_ptr->objects[i].descriptor.data.data.empty() ) {
	attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(object_ptr->objects[i].descriptor) );
      }
      attributes[POSITION] = AttributePtr( new PositionAttribute(object_ptr->objects[i].position) );
      attributes[SIZE] = AttributePtr( new SizeAttribute(object_ptr->objects[i].size) );
    } catch( const std::exception &e ) {
      ROS_ERROR("[AnchorManagement::match] %s", e.what() );
      continue;
    }

    // Create a map of all object attributes
    PerceptMap percepts;
    try {
      percepts[SPATIAL] = PerceptPtr( new SpatialPercept(object_ptr->objects[i].spatial) );
      percepts[VISUAL] = PerceptPtr( new VisualPercept(object_ptr->objects[i].visual) );
    } catch( const std::exception &e ) {
      ROS_ERROR("[AnchorManagement::match] %s", e.what() );
      continue;
    }

    // Create the object
    ObjectMatching object(attributes, percepts);

    // Match all attributes
    this->_anchors->match( object._attributes, object._matches);

    // Process the matches
    for( auto ite = object._matches.begin(); ite != object._matches.end(); ++ite) {
      cv::Mat sample( 1, 5, CV_32F);
      sample.at<float>( 0, 0) = ite->second[CATEGORY];
      sample.at<float>( 0, 1) = ite->second[COLOR];
      sample.at<float>( 0, 2) = ite->second[POSITION];
      sample.at<float>( 0, 3) = ite->second[SIZE];
      sample.at<float>( 0, 4) = (float)(2.0 / (1.0 + exp( abs(this->_anchors->diff( ite->first, t)) )));

      // Classify the sample
      float pred = this->_classifier->predict(sample);
      if( pred > 0.65 ) {
	//cout << ite->first << " - " << pred << endl;
	object._predictions.insert( pair<string, float>( ite->first, pred) );
      }
      //cout << "---" << endl;
    }

    // Add to list...
    objects.push_back(std::move(object));
  }

  /*
  // Globally process all the matches
  for( uint i = 0; i < objects.size() - 1; i++) {
    for( uint j = i + 1; j < objects.size(); j++) {
      objects[i].filter(objects[j]);
    }
  }
  */

  // Manage the anchors
  for( uint i = 0; i < objects.size(); i++) {
    string id = objects[i].getId();
    if( !id.empty() ) {
      this->_anchors->re_acquire( id, objects[i]._attributes, objects[i]._percepts, t ); // RE_ACQUIRE
    }
    else {
      this->_anchors->acquire(objects[i]._attributes, objects[i]._percepts, t); // ACQUIRE
    }
  }

  // Get all anchors seen in the scene at this time (t)
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

  //ROS_INFO("Anchors: %d", (int)this->_anchors->size());
  //ROS_INFO("Time: %.2f", t.toSec() - _time_zero);
  //ROS_INFO("------------------------------");
}



/* -----------------------------------------
   Main track function(s)
   --------------------------------------- */
void AnchorManagement::track( const anchor_msgs::LogicAnchorArrayPtr &track_ptr ) {
  
  // Maintain all incoming tracked objects
  ros::Time t = track_ptr->header.stamp;
  for( uint i = 0; i < track_ptr->anchors.size(); i++) {

    //ROS_WARN("Anchor id: %s", track_ptr->anchors[i].id.c_str());
	     
    // Tracked hidden object, add the position(s) of hte object
    if( !track_ptr->anchors[i].observed ) {

      if( this->_anchors->exist(track_ptr->anchors[i].id) ) {

	// Populate a list of position messages
	std::vector<geometry_msgs::PoseStamped> array;
	auto p = track_ptr->anchors[i].particle_positions.begin();
	for( ; p != track_ptr->anchors[i].particle_positions.end(); ++p ) {  
	  array.push_back(p->data);
	}

	// Track the anchor
	anchoring::AttributeMap attributes;
	attributes[POSITION] = AttributePtr( new PositionAttribute(array) );
	this->_anchors->track( track_ptr->anchors[i].id, attributes, t); // TRACK
      }
    }
  }

}


/* -----------------------------------------

   Update function
   --------------------
   Updates and anchor based on the information
   given by language instruction(s).

   ------------------------------------------- */
void AnchorManagement::update( const anchor_msgs::SemanticAnchorArrayPtr &update_ptr ) {
  ROS_ERROR("Anchoring: got a semantic update!");
  
  // Maintain all incoming tracked objects
  ros::Time t = ros::Time::now();
  for( uint i = 0; i < update_ptr->anchors.size(); i++) {

    // Update the class categories of an anchor
    anchoring::AttributeMap attributes;
    attributes[CATEGORY] = AttributePtr( new CategoryAttribute(update_ptr->anchors[i].category) );
    this->_anchors->track( update_ptr->anchors[i].id, attributes, t); // TRACK
    //this->_anchors->re_acquire( update_ptr->anchors[i].id, attributes, t); // TRACK
  }
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

    // Remove expired anchors from list (while idle)
    if( !this->_anchors->empty() ) {

      anchor_msgs::RemovedAnchorArray msg;
      this->_anchors->remove(msg.ids);

      // Publish id's of removed anchors
      if( !msg.ids.empty() ) {
	this->_remove_pub.publish(msg);
      }
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


// ---[ OLD STUFF ]---------

/* -----------------------------------------
   Predict if
   ----------------------------------------
float AnchorManagement::predict(map< string, map<anchoring::AttributeType, float> > &matches) {

}
   --------------------------------------- */
/*
// ---[ Track method based on data association ]---
void AnchorManagement::associate( const anchor_msgs::LogicAssociationArrayConstPtr &associations_ptr ) {

  //ROS_INFO("Got associations.");

  try {

    // Update (merge) anchors based on probabilistic object tracking
    for( auto &msg: associations_ptr->associations) {
      int idx_id = -1;
      int idx_best = -1;
      float best = 0.0;
      //std::cout << "Id: " << this->_anchors->toString(msg.id) << std::endl;
      for( uint i = 0; i < msg.associations.size(); i++) {
	//std::cout << "Assoc: " << this->_anchors->toString(msg.associations[i]);
	//std::cout << " (" << msg.probabilities[i] << ")" << std::endl;
	if( msg.probabilities[i] > best ) {
	  best = msg.probabilities[i];
	  idx_best = i;
	}
	if(msg.id == msg.associations[i]){
	  idx_id = i;
	}
      }

      //if( msg.associations[idx] != msg.id && best > 0.5 ) {
      if (idx_id == -1) {
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
*/


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
   -------------------------------------- 
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

    // ( DESCRIPTOR OR CATEGORY ) AND ( COLOR AND SIZE )
    float rate_1 = max( ite->second[DESCRIPTOR], ite->second[CATEGORY] );
    float rate_2 = min( ite->second[COLOR], ite->second[SIZE] );
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
----------------------------------*/

// ---[ END ]----
