
#include <algorithm>
#include <cassert>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <anchoring/percept.hpp>

// Includes for image compression
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace anchoring {

  // Used namespace(s)
  using namespace std;

  // --[ Namspace function(s) ]--  
  PerceptType mapPerceptType(const string &type) {
    return (type == "spatial" ? SPATIAL : VISUAL);
  }

  // Create an attribute (pointer) based on the attribute type  
  PerceptPtr createPercept(PerceptType type) {
    switch(type) {
    case SPATIAL: return PerceptPtr( new SpatialPercept(type) );
    default:      return PerceptPtr( new VisualPercept(type) );
    };
  }
  
  // -------------------------------
  // Visual percept class methods
  // ----------------------------------
  VisualPercept:: VisualPercept( const anchor_msgs::VisualPercept &msg,
				 PerceptType type ) : PerceptCommon(type) {
    // Read the data from ROS msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      if( sizeof(msg.data) / sizeof(unsigned char) > 0 ) {
	cv_ptr = cv_bridge::toCvCopy( msg.data,
				      sensor_msgs::image_encodings::BGR8 );
	cv_ptr->image.copyTo(this->_data);
      }
    } catch (cv_bridge::Exception& e) {
      // FIX THIS SHIT LATER ON!!
      throw std::logic_error("[VisualPercept:: VisualPercept]:" + std::string(e.what()) );
    }    
    this->_border = msg.border;
    this->_point = msg.point;
  }

  mongo::Database::Document VisualPercept::serialize() {
    mongo::Database::Document doc;

    // Compress the image  
    vector<uchar> buff;
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);
    cv::imencode( ".jpg", this->_data, buff, compression_params);

    // Save the image (to DB)
    try {
      std::size_t length = buff.size();
      doc.add<unsigned char*>( "data", buff.data(), length);
      
      // Save the contour
      for( auto &point : _border.contour ) {
	mongo::Database::Document p_doc;
	p_doc.add<int>( "x", point.x);
	p_doc.add<int>( "y", point.y);
	doc.append( "border", p_doc);
      }

      // Save the upper corner point of the image
      mongo::Database::Document p_doc;
      p_doc.add<int>( "x", this->_point.x);
      p_doc.add<int>( "y", this->_point.y);
      doc.add( "point", p_doc);
    }
    catch( const std::exception &e) {
      cout << "[VisualPercept::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void VisualPercept::deserialize(const mongo::Database::Document &doc) {
  
    // Load the image 
    try {
      std::size_t length = doc.get<std::size_t>("data");
      unsigned char* array = doc.get<unsigned char*>("data");

      // Decompress the image      
      vector<uchar> buff( array, array + length);
      this->_data = cv::imdecode( cv::Mat(buff), CV_LOAD_IMAGE_COLOR);

      // Load the contour
      for( mongo::document_iterator ite = doc.begin("border"); ite != doc.end("border"); ++ite) {
	anchor_msgs::Point2d point;
	point.x = ite->get<int>("x");
	point.y = ite->get<int>("y");
	this->_border.contour.push_back(point);	
      }

      // Load the upper corner point of the image
      anchor_msgs::Point2d point;
      this->_point.x = doc.get("point").get<int>("x");
      this->_point.y = doc.get("point").get<int>("y");
    }
    catch( const std::exception &e) {
      cout << "[VisualPercept::deserialize]" << e.what() << endl;
    }
  }

  void VisualPercept::populate(anchor_msgs::Display &msg) {
    msg.border = this->_border;
  }

  bool VisualPercept::update(const unique_ptr<PerceptCommon> &new_ptr) {
    
    // Typecast the new percept pointer
    VisualPercept *raw_ptr = dynamic_cast<VisualPercept*>(new_ptr.get());
    assert( raw_ptr != nullptr );

    // Update the visual data
    if( !raw_ptr->_data.empty() ) {
      this->_data = raw_ptr->_data;
      this->_border = raw_ptr->_border;
      this->_point = raw_ptr->_point;
    }
    return true;
  }

} // namespace anchoring
