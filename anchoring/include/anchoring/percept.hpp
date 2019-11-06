#ifndef __PERCEPT_HPP__
#define __PERCEPT_HPP__

#include <iostream>
#include <vector>
#include <utility>
#include <memory>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// Anchoring includes
#include <anchor_msgs/Anchor.h>  // ...includes ALL perrcepts messages
#include <anchor_msgs/Display.h>

#include <database/database.hpp>

// --[ anchoring namespace... 
namespace anchoring {
  
  using namespace std;

  // ---[ Different attribute types ]---
  enum PerceptType {
    SPATIAL    = 0,
    VISUAL     = 1
  };
  
    
  // ---[ Common attribute base struct 
  // -----------------------------------------------
  struct PerceptCommon {
    PerceptType _type;

    // Constructor/destrcutor
    PerceptCommon( PerceptType type) : _type(type) {}
    virtual ~PerceptCommon() {}

    // Virtual database methods
    virtual mongo::Database::Document serialize() {} 
    virtual void deserialize(const mongo::Database::Document &doc) {}
    virtual void populate(anchor_msgs::Anchor &msg) {}
    virtual void populate(anchor_msgs::Display &msg) {}

    // Virtual update methods (with default dummy)
    virtual bool update(const unique_ptr<PerceptCommon> &new_ptr) { return false; }  

    string getTypeStr() { return (this->_type == SPATIAL ? "spatial" : "visual"); } 
  }; // ...base attribute struct ]---
  

  // ---[ Type defined attribute pointer containers ]---    
  //------------------------------------------------------------
  typedef unique_ptr<PerceptCommon> PerceptPtr;
  typedef map<PerceptType, PerceptPtr> PerceptMap;
  
  
  // ---[ Spatial percept struct ]---
  //-----------------------------------------------
  struct SpatialPercept : public PerceptCommon {
    //cv::Mat _data;

    // Constructors
    SpatialPercept(PerceptType type = SPATIAL) : PerceptCommon(type) {}
    SpatialPercept( const anchor_msgs::SpatialPercept &msg, 
		    PerceptType type = SPATIAL ) : PerceptCommon(type) {}
    /*
    // Overrided methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    void populate(anchor_msgs::Display &msg);
    bool update(const PerceptPtr &new_ptr);
    */
  }; 

  
  // ---[ Visual percept struct ]---
  //-----------------------------------------------
  struct VisualPercept : public PerceptCommon {
    cv::Mat _data;
    anchor_msgs::Contour _border;
    anchor_msgs::Point2d _point;

    // Constructors
    VisualPercept(PerceptType type = VISUAL) : PerceptCommon(type) {}
    VisualPercept( const anchor_msgs::VisualPercept &msg, 
		   PerceptType type = VISUAL );

    // Overrided methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    void populate(anchor_msgs::Display &msg);
    bool update(const PerceptPtr &new_ptr);
  };   


  // --[ Namspace function headers (and template functions) ]--
  PerceptType mapPerceptType(const string &type);
  PerceptPtr createPercept(PerceptType type);
  
} // namespace anchoring ]---
  
#endif // __PERCEPT_HPP__ 
