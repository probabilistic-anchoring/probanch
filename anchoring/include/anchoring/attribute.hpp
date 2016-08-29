#ifndef __ATTRIBUTE_HPP__
#define __ATTRIBUTE_HPP__

#include <iostream>
#include <vector>
#include <utility>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <anchoring/database.hpp>

namespace anchoring {

  using namespace std;

  /**
   * Different attribute types 
   */
  enum AttributeType {
    DESCRIPTOR = 0,
    COLOR      = 1,
    SHAPE      = 2,
    LOCATION   = 3,
    CAFFE      = 4
  };

  /**
   * Common attribute base struct
   * -----------------------------------------------
   */
  struct AttributeCommon {
    vector<string> _symbols;
    AttributeType _type;

    // Constructor/destrcutor
    AttributeCommon( AttributeType type) : _type(type) {}
    AttributeCommon( vector<string> symbols, AttributeType type) : _symbols(symbols), _type(type) {}
    virtual ~AttributeCommon() {}

    // Virtual database methods
    virtual void serialize(MongoDatabase::Subdoc &db_sub); 
    virtual void deserialize(const MongoDatabase::Subdoc &db_sub);
    
    // Virtual match method
    virtual float match(const unique_ptr<AttributeCommon> &query_ptr) = 0;

    // Virtual append and update methods (with default dummy)
    virtual void append(const unique_ptr<AttributeCommon> &query_ptr) { }
    virtual void update(const unique_ptr<AttributeCommon> &query_ptr) { }  

    string getTypeStr();
  };
  
  // Type defined attribute pointer containers    
  typedef unique_ptr<AttributeCommon> AttributePtr;
  typedef map<AttributeType, AttributePtr> AttributeMap;
  typedef map<AttributeType, float> MatchMap;
  

  /**
   * Color attribute struct
   * -----------------------------------------------
   */
  struct ColorAttribute : public AttributeCommon {
    int _total;
    cv::Mat _data;
 
    // Constructors
    ColorAttribute(AttributeType type = COLOR) : AttributeCommon(type) {}
    ColorAttribute( const vector<uint16_t> &data, 
		    const vector<string> &symbols = vector<string>(),
		    AttributeType type = COLOR );
    ColorAttribute( const cv::Mat &data, 
		    const vector<string> &symbols = vector<string>(),
		    AttributeType type = COLOR )  : AttributeCommon( symbols, type) { 
      this->_data = data;
    }    

    // Override methods
    void serialize(MongoDatabase::Subdoc &db_sub); 
    void deserialize(const MongoDatabase::Subdoc &db_sub);
    float match(const AttributePtr &query_ptr);
  }; 
  

  /**
   * Descriptor attribute struct
   * -----------------------------------------------
   */
  struct DescriptorAttribute : public AttributeCommon {
    cv::Mat _data;

    // Constructors
    DescriptorAttribute(AttributeType type = DESCRIPTOR) : AttributeCommon(type) {}
    DescriptorAttribute( const cv::Mat &data, 
			 const vector<string> &symbols = vector<string>(),
			 AttributeType type = DESCRIPTOR ) : AttributeCommon( symbols, type) {
      // (Deep) copy of the data
      data.copyTo(this->_data);
    }

    // Override methods
    void serialize(MongoDatabase::Subdoc &db_sub); 
    void deserialize(const MongoDatabase::Subdoc &db_sub);
    virtual float match(const AttributePtr &query_ptr) { return 0.0; }
  }; 
  
  /**
   * Location attribute struct
   * -----------------------------------------------
   */
  struct LocationAttribute : public AttributeCommon {
    vector<geometry_msgs::PoseStamped> _array;  

    // Constructors
    LocationAttribute(AttributeType type = LOCATION) : AttributeCommon(type) {}
    LocationAttribute( const geometry_msgs::PoseStamped &data, 
		       const vector<string> &symbols = vector<string>(),
		       AttributeType type = LOCATION ) : AttributeCommon( symbols, type) {
      // Add the data (including a timestamp)
      this->_array.push_back(data);
    }
    
    // Override methods
    void serialize(MongoDatabase::Subdoc &db_sub); 
    void deserialize(const MongoDatabase::Subdoc &db_sub);
    float match(const AttributePtr &query_ptr);
    void append(const unique_ptr<AttributeCommon> &query_ptr);
    void update(const unique_ptr<AttributeCommon> &query_ptr);
  }; 
  
  /**
   * Shape attribute struct
   * -----------------------------------------------
   */
  struct ShapeAttribute : public AttributeCommon {
    geometry_msgs::Point _data;

    // Constructors
    ShapeAttribute(AttributeType type = SHAPE) : AttributeCommon(type) {}
    ShapeAttribute( const geometry_msgs::Point &data, 
		    const vector<string> &symbols = vector<string>(),
		    AttributeType type = SHAPE ) : AttributeCommon( symbols, type) {
      this->_data = data;
    }

    // Override methods
    void serialize(MongoDatabase::Subdoc &db_sub); 
    void deserialize(const MongoDatabase::Subdoc &db_sub);
    float match(const AttributePtr &query_ptr);
  }; 

  /**
   * Caffe attribute struct
   * -----------------------------------------------
   */
  struct CaffeAttribute : public AttributeCommon {
    cv::Mat _data;
    vector<double> _predictions;
 
    // Constructors
    CaffeAttribute(AttributeType type = CAFFE) : AttributeCommon(type) {}
    CaffeAttribute( const cv::Mat &data,
		    const vector<float> &predictions, 
		    const vector<string> &symbols = vector<string>(),
		    AttributeType type = CAFFE ) : AttributeCommon( symbols, type) {
      data.copyTo(this->_data);
      this->_predictions = vector<double>( predictions.begin(), predictions.end() );
    }  

    // Override methods
    void serialize(MongoDatabase::Subdoc &db_sub); 
    void deserialize(const MongoDatabase::Subdoc &db_sub);
    float match(const AttributePtr &query_ptr);
  }; 

} // namespace anchoring
  
#endif // __ATTRIBUTE_HPP__ 
