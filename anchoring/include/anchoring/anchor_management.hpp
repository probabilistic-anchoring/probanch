#ifndef __ANCHOR_MANAGEMENT_HPP__
#define __ANCHOR_MANAGEMENT_HPP__

#include <anchor_msgs/ObjectArray.h>
#include <anchor_msgs/MovementArray.h>
#include <anchor_msgs/TimedRequest.h>
#include <anchor_msgs/SpatialRequest.h>

#include <anchor_msgs/LogicAnchorArray.h>

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
  machine::MachinePtr _classifier;

  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh, _priv_nh;
  ros::Subscriber _object_sub, _track_sub, _assoc_sub, _update_sub;
  ros::ServiceServer _spatial_srv, _timed_srv;
  ros::Publisher _anchor_pub, _display_pub;

  double _time_zero;

  /* Struct for temporary storage of matching results */
  struct ObjectMatching {
    ObjectMatching(anchoring::AttributeMap &attributes) : _attributes(std::move(attributes)) {}
    /*
    ObjectMatching( ObjectMatching&& other ) :
      _attributes(std::move(other._attributes)),
      _matches(other._matches),
      _predictions(other._predictions)
    {
    }
    ObjectMatching& operator=(ObjectMatching &&other) {
      _attributes = std::move(other._attributes);
      _matches = other._matches;
      _predictions = other._predictions;
      return *this;
    }
    */

    // Local variables
    anchoring::AttributeMap _attributes;
    map< string, map<anchoring::AttributeType, float> > _matches;
    map< string, float> _predictions;

    // Match attributes of an unknown object (anchor)
    void match(ObjectMatching &other) {
      /*
      for( auto ite = other._attributes.begin(); ite != other._attributes.end(); ++ite) {
	if( this->_attributes.find(ite->first) != this->_attributes.end() ) {
	  std::pair<AttributeType, float> rate;
	  rate.first = ite->first;	
	  rate.second = this->_attributes[ite->first]->match(ite->second);
	  result.insert(rate);
	}
      }
      */
    }
    
    // Filter classification predictions
    void filter(ObjectMatching &other) {
      for( auto ite = this->_predictions.begin(); ite != this->_predictions.end(); ++ite) {
	if( other._predictions.find(ite->first) != other._predictions.end() ) {
	  if( ite->second > other._predictions[ite->first] ) {
	    other._predictions.erase(ite->first);
	  }
	  else {
	    this->_predictions.erase(ite);
	  }
	}
      }
    }

    // Get the id of the best matching anchor
    string getId() {
      string id = "";
      float best = 0.0;
      for( auto ite = this->_predictions.begin(); ite != this->_predictions.end(); ++ite) {
	if( ite->second > best ) {
	  best = ite->second;
	  id = ite->first;
	}
      }
      return id;
    }
  };

  // Main anchoring management fucntions
  void match( const anchor_msgs::ObjectArrayConstPtr &object_ptr );
  void track( const anchor_msgs::LogicAnchorArrayPtr &track_ptr );
  void update( const anchor_msgs::LogicAnchorPtr &update_ptr );
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

/* ---[ OLD STUFF ] -----------------

  float predict(map< string, map<anchoring::AttributeType, float> > &matches);
  void associate( const anchor_msgs::LogicAssociationArrayConstPtr &associations_ptr);
  int process( map< string, map<anchoring::AttributeType, float> > &matches,
	       string &id,
	       float dist_th,
	       float rate_th = 0.75 );
  ---------------------------------- */
