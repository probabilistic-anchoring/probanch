#ifndef __ATTRIBUTE_HPP__
#define __ATTRIBUTE_HPP__

#include <iostream>
#include <vector>
#include <utility>
#include <memory>

// ROS includes
#include <ros/ros.h>

// OpenCV includes
#include <opencv2/core/core.hpp>

// Anchor includes
#include <anchor_msgs/Contour.h>
#include <anchor_msgs/Display.h>
#include <anchor_msgs/Anchor.h>

#include <anchor_utils/database.hpp>

// --[ anchoring namespace... 
namespace anchoring {

  using namespace std;

  // ---[ Different attribute types ]---
  enum AttributeType {
    DESCRIPTOR = 0,
    COLOR      = 1,
    SHAPE      = 2,
    POSITION   = 3,
    CAFFE      = 4
  };

  // ---[ Common attribute base struct 
  // -----------------------------------------------
  struct AttributeCommon {
    vector<std::string> _symbols;
    AttributeType _type;

    // Constructor/destrcutor
    AttributeCommon( AttributeType type) : _type(type) {}
    AttributeCommon( vector<string> symbols, AttributeType type) : _symbols(symbols), _type(type) {}
    virtual ~AttributeCommon() {}

    // Virtual database methods
    virtual mongo::Database::Document serialize(); 
    virtual void deserialize(const mongo::Database::Document &doc);
    virtual void populate(anchor_msgs::Display &msg) { }
    virtual void populate(anchor_msgs::Anchor &msg) { }
    
    // Virtual match method
    virtual float match(const unique_ptr<AttributeCommon> &query_ptr) = 0;

    // Virtual update methods (with default dummy)
    virtual bool update(const unique_ptr<AttributeCommon> &new_ptr) { return false; }  
    virtual string toString() { return ""; }

    string getTypeStr();

  }; // ...base attribute struct ]---
  

  // ---[ Type defined attribute pointer containers ]---    
  //------------------------------------------------------------
  typedef unique_ptr<AttributeCommon> AttributePtr;
  typedef map<AttributeType, AttributePtr> AttributeMap;
  typedef map<AttributeType, float> MatchMap;
  
  
  // ---[ Color attribute struct ]---
  //-----------------------------------------------
  struct ColorAttribute : public AttributeCommon {
    cv::Mat _data;
    vector<double> _predictions;
    float _n;

    // Constructors
    ColorAttribute(AttributeType type = COLOR) : AttributeCommon(type) {}
    ColorAttribute( const anchor_msgs::ColorAttribute &msg, 
		    AttributeType type = COLOR );

    // Override methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    void populate(anchor_msgs::Anchor &msg);
    void populate(anchor_msgs::Display &msg);
    float match(const AttributePtr &query_ptr);
    bool update(const AttributePtr &new_ptr);
    string toString();
  }; 
  

  // ---[ Descriptor attribute struct ]---
  //-----------------------------------------------
  struct DescriptorAttribute : public AttributeCommon {
    cv::Mat _data;

    // Constructors
    DescriptorAttribute(AttributeType type = DESCRIPTOR) : AttributeCommon(type) {}
    DescriptorAttribute( const anchor_msgs::DescriptorAttribute &msg, 
			 AttributeType type = DESCRIPTOR );

    // Override methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    virtual float match(const AttributePtr &query_ptr) { return 0.0; }
  }; 
  

  // ---[ Position attribute struct ]---
  //-----------------------------------------------
  struct PositionAttribute : public AttributeCommon {
    vector<geometry_msgs::PoseStamped> _array;  

    // Constructors
    PositionAttribute(AttributeType type = POSITION) : AttributeCommon(type) {}
    PositionAttribute( const anchor_msgs::PositionAttribute &msg,
		       AttributeType type = POSITION );
    PositionAttribute( const geometry_msgs::PoseStamped &data, 
		       const vector<string> &symbols = vector<string>(),
		       AttributeType type = POSITION ) : AttributeCommon( symbols, type) {
      // Add the data (including a timestamp)
      this->_array.push_back(data);
    }
    PositionAttribute( const AttributePtr &other_ptr,
		       AttributeType type = POSITION);
    
    // Override methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    void populate(anchor_msgs::Anchor &msg);
    void populate(anchor_msgs::Display &msg);
    float match(const AttributePtr &query_ptr);
    bool update(const unique_ptr<AttributeCommon> &new_ptr);
    string toString();
  }; 
  

  // ---[ Shape attribute struct ]---
  //-----------------------------------------------
  struct ShapeAttribute : public AttributeCommon {
    geometry_msgs::Point _data;

    // Constructors
    ShapeAttribute(AttributeType type = SHAPE) : AttributeCommon(type) {}
    ShapeAttribute( const anchor_msgs::ShapeAttribute &msg,
		    AttributeType type = SHAPE);

    // Override methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    void populate(anchor_msgs::Anchor &msg);
    void populate(anchor_msgs::Display &msg);
    float match(const AttributePtr &query_ptr);
    string toString();
  }; 


  // ---[ Caffe attribute struct ]---
  //-----------------------------------------------
  struct CaffeAttribute : public AttributeCommon {
    cv::Mat _data;
    vector<double> _predictions;
    vector<double> _N;
    
    anchor_msgs::Contour _border;
    anchor_msgs::Point2d _point;

    // Constructors
    CaffeAttribute(AttributeType type = CAFFE) : AttributeCommon(type) {}
    CaffeAttribute( const anchor_msgs::CaffeAttribute &msg,
		    AttributeType type = CAFFE );

    // Override methods
    mongo::Database::Document serialize(); 
    void deserialize(const mongo::Database::Document &doc);
    void populate(anchor_msgs::Anchor &msg);
    void populate(anchor_msgs::Display &msg);
    float match(const AttributePtr &query_ptr);
    bool update(const unique_ptr<AttributeCommon> &new_ptr);
    string toString();
  }; 

} // namespace anchoring ]---
  
#endif // __ATTRIBUTE_HPP__ 
