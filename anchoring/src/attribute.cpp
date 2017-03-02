
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #define NDEBUG
#include <cmath>
#include <cassert>

#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>

#include <anchoring/attribute.hpp>

namespace anchoring {

  using namespace std;

  // ---------------------------------------
  // Common attribute base struct methods
  // ------------------------------------------
  mongo::Database::Document AttributeCommon::serialize() {
    mongo::Database::Document doc;

    // Save the symbols
    try {
      doc.add<string>( "symbols", this->_symbols);
    }
    catch( const std::exception &e) {
      cout << "[AttributeCommon::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void AttributeCommon::deserialize(const mongo::Database::Document &doc) {

    // Load the symbols
    try {
      doc.get<string>( "symbols", this->_symbols);
    }
    catch( const std::exception &e) {
      cout << "[AttributeCommon::deserialize]" << e.what() << endl;
    }
  }

  // Symbol mapping of the attribute type
  string AttributeCommon::getTypeStr() {
    string result;
    switch(this->_type) {
    case DESCRIPTOR: result = "descriptor"; break;
    case COLOR:      result = "color"; break;
    case SHAPE:      result = "shape"; break;
    case POSITION:   result = "position"; break;
    default:         result = "image"; break;
    }
    return result;
  }


  // --------------------------------
  // Color attribute methods
  // ------------------------------------  
  ColorAttribute::ColorAttribute( const anchor_msgs::ColorAttribute &msg, 
				  AttributeType type ) : AttributeCommon( msg.symbols, type) {
    // Read the data from ROS msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( msg.data,
				    sensor_msgs::image_encodings::TYPE_32FC1 );
      cv_ptr->image.copyTo(this->_data);
    } catch (cv_bridge::Exception& e) {
      throw std::logic_error("[ColorAttribute::ColorAttribute]:" + std::string(e.what()) );
    }
    this->_predictions = vector<double>( msg.predictions.begin(), msg.predictions.end() );
  }

  mongo::Database::Document ColorAttribute::serialize() {
  
    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();

    // Save the color
    try {      
      vector<double> array;      
      for( int i = 0; i < this->_data.cols; i++) {
	array.push_back(this->_data.at<float>( 0, i));
      }
      doc.add<double>( "data", array);

      // Save (color) predictions
      doc.add<double>( "predictions", this->_predictions);
    }
    catch( const std::exception &e) {
      cout << "[ColorAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void ColorAttribute::deserialize(const mongo::Database::Document &doc) {

    // Load the color 
    try {
      
      vector<double> array;
      doc.get<double>( "data", array);
      this->_data = cv::Mat( 1, array.size(), CV_32F, &array.front());

      // Load (color) predictions
      doc.get<double>( "predictions", this->_predictions);
    }
    catch( const std::exception &e) {
      cout << "[ColorAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }

  void ColorAttribute::populate(anchor_msgs::Anchor &msg) {
    anchor_msgs::ColorAttribute color_msg;
    color_msg.symbols = this->_symbols;
    for( auto ite = this->_predictions.begin(); ite != this->_predictions.end(); ++ite) {
      color_msg.predictions.push_back((float)*ite);
    }
    msg.color = color_msg;
  }
  void ColorAttribute::populate(anchor_msgs::Display &msg) {
    msg.colors = this->_symbols;
  }

  
  float ColorAttribute::match(const AttributePtr &query_ptr) {
    
    // Typecast the query pointer
    ColorAttribute *raw_ptr = dynamic_cast<ColorAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    return cv::compareHist( raw_ptr->_data, this->_data, CV_COMP_INTERSECT); // CV_COMP_CORREL
  }

  string ColorAttribute::toString() {
    std::stringstream ss;
    ss << "[ ";
    for( size_t i = 0; i < _symbols.size(); i++ ) {
      ss << _symbols[i];
      if( i + 1 < _symbols.size() ) {
	ss << ", ";
      }
    }
    ss << "]";
    return ss.str();
  }


  // ------------------------------------
  // Descriptor attribute methods
  // --------------------------------------------
  DescriptorAttribute::DescriptorAttribute( const anchor_msgs::DescriptorAttribute &msg, 
					    AttributeType type ) : AttributeCommon( msg.symbols, type) {
    // Read the data from ROS msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( msg.data, 
				    sensor_msgs::image_encodings::MONO8 );
      // (Deep) copy of the data
      cv_ptr->image.copyTo(this->_data);
    } catch (cv_bridge::Exception& e) {
      throw std::logic_error("[DescriptorAttribute::DescriptorAttribute]:" + std::string(e.what()) );
    }
  }

  mongo::Database::Document DescriptorAttribute::serialize() {

    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();
  
    // Save the descriptor
    try {
      doc.add<int>( "rows", (int)this->_data.rows);
      doc.add<int>( "cols", (int)this->_data.cols);

      std::size_t length = this->_data.rows * this->_data.cols;
      doc.add<unsigned char*>( "data", this->_data.data, length);
    }
    catch( const std::exception &e) {
      cout << "[DescriptorAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void DescriptorAttribute::deserialize(const mongo::Database::Document &doc) {

    // Load the descriptor
    try {
      int rows = doc.get<int>( "rows");
      int cols = doc.get<int>( "cols");
      unsigned char* array = doc.get<unsigned char*>( "data");
      this->_data = cv::Mat( rows, cols, CV_8U, array);
    }
    catch( const std::exception &e) {
      cout << "[DescriptorAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }


  // ----------------------------------
  // Position attribute methods
  // ---------------------------------------
  PositionAttribute::PositionAttribute( const anchor_msgs::PositionAttribute &msg,
					AttributeType type ) : AttributeCommon( msg.symbols, type) {
    // Add the data (including a timestamp)
    this->_array.push_back(msg.data);
  }

  mongo::Database::Document PositionAttribute::serialize() {

    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();
    
    // Save the location
    try {
      
      vector<geometry_msgs::PoseStamped>::iterator ite_d = this->_array.begin();
      for( ; ite_d != this->_array.end(); ++ite_d ) {  

	mongo::Database::Document subdoc;

	// Add time
	subdoc.add<double>( "t", ite_d->header.stamp.toSec());

	// Add position
	mongo::Database::Document position;
	position.add<double>( "x", ite_d->pose.position.x);
	position.add<double>( "y", ite_d->pose.position.y);
	position.add<double>( "z", ite_d->pose.position.z);
	subdoc.add( "position", position);

	// Add orientation
	mongo::Database::Document orientation;
	orientation.add<double>( "x", ite_d->pose.orientation.x);
	orientation.add<double>( "y", ite_d->pose.orientation.y);
	orientation.add<double>( "z", ite_d->pose.orientation.z);
	orientation.add<double>( "w", ite_d->pose.orientation.w);
	subdoc.add( "orientation", orientation);

	doc.append( "array", subdoc);
      }
    }
    catch( const std::exception &e) {
      cout << "[PositionAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void PositionAttribute::deserialize(const mongo::Database::Document &doc) {
    
    // Load the positions
    try {
      
      // Read a series of position data
      for( mongo::document_iterator ite = doc.begin("array"); ite != doc.end("array"); ++ite) {
	geometry_msgs::PoseStamped data;

	// Read the time
	data.header.stamp = ros::Time(ite->get<double>("t"));	

	// Read the symbol
	//string symbol = sub.get<string>("symbol");
	//this->_symbols.push_back(symbol);
	
	// Read position
	mongo::Database::Document position = ite->get("position");
	data.pose.position.x = position.get<double>("x");
	data.pose.position.y = position.get<double>("y");
	data.pose.position.z = position.get<double>("z");

	// Read orientation
	mongo::Database::Document orientation = ite->get("orientation");
	data.pose.orientation.x = orientation.get<double>("x");
	data.pose.orientation.y = orientation.get<double>("y");
	data.pose.orientation.z = orientation.get<double>("z");
	data.pose.orientation.w = orientation.get<double>("w");
	
	this->_array.push_back(data);
      }
    }
    catch( const std::exception &e) {
      cout << "[PositionAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }

  void PositionAttribute::populate(anchor_msgs::Anchor &msg) {
    anchor_msgs::PositionAttribute pose_msg;
    pose_msg.data = this->_array.back();
    msg.position = pose_msg;
  }

  void PositionAttribute::populate(anchor_msgs::Display &msg) {
    msg.pos = this->_array.back();
  }
  
  float PositionAttribute::match(const AttributePtr &query_ptr) {

    // Typecast the query pointer
    PositionAttribute *raw_ptr = dynamic_cast<PositionAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Match aginst the last poistion (only)
    geometry_msgs::PoseStamped train = this->_array.back();
    geometry_msgs::PoseStamped query = raw_ptr->_array.back();

    // 1. Check the time
    if( query.header.stamp.toSec() - train.header.stamp.toSec() > 1.0 ) {
      return 0.0;
    }

    // 2. Check the distance
    double diff_x = query.pose.position.x - train.pose.position.x;
    double diff_y = query.pose.position.y - train.pose.position.y;
    double diff_z = query.pose.position.z - train.pose.position.z;
    double dist  = sqrt( diff_x*diff_x + diff_y*diff_y + diff_z*diff_z );

    return 1.0 / exp(dist);
  }
  
  /*
  void PositionAttribute::append(const unique_ptr<AttributeCommon> &query_ptr) {
    
    // Typecast the query pointer
    PositionAttribute *raw_ptr = dynamic_cast<PositionAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Append the location (including a timestamp)
    this->_array.push_back(raw_ptr->_array.front());

    // Append the symbol (if there exists an symbol)
    if( !raw_ptr->_symbols.empty() ) {
      this->_symbols.push_back(raw_ptr->_symbols.front());
    }  
  }
  */

  bool PositionAttribute::update(const unique_ptr<AttributeCommon> &query_ptr) {
    
    // Typecast the query pointer
    PositionAttribute *raw_ptr = dynamic_cast<PositionAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Update the location
    this->_array.back().header.stamp = raw_ptr->_array.front().header.stamp;
    /*
    this->_array.back() = raw_ptr->_array.front();
    if( !raw_ptr->_symbols.empty() ) {
      this->_symbols.back() = raw_ptr->_symbols.front();
    }
    */
    return true;
  }

  string PositionAttribute::toString() {
    std::stringstream ss;
    ss << "@ {";
    ss << " x = " << this->_array.back().pose.position.x;
    ss << ", y = " << this->_array.back().pose.position.y;
    ss << ", z = " << this->_array.back().pose.position.z;
    ss << "}";
    return ss.str();
  }

  
  // --------------------------------
  // Shape attribute class methods
  // -------------------------------------
  ShapeAttribute::ShapeAttribute( const anchor_msgs::ShapeAttribute &msg,
				  AttributeType type ) : AttributeCommon( msg.symbols, type) {
    this->_data = msg.data;
  }

  mongo::Database::Document ShapeAttribute::serialize() {
  
    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();

    // Save the shape
    try {
      std::vector<double> shape;
      shape.push_back(this->_data.x);
      shape.push_back(this->_data.y);
      shape.push_back(this->_data.z);
      doc.add<double>( "data", shape);
    }
    catch( const std::exception &e) {
      cout << "[ShapeAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void ShapeAttribute::deserialize(const mongo::Database::Document &doc) {

    // Load the shape
    try {
      vector<double> shape;
      doc.get<double>( "data", shape);
      this->_data.x = shape[0];
      this->_data.y = shape[1];
      this->_data.z = shape[2];
    }
    catch( const std::exception &e) {
      cout << "[ShapeAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }

  void ShapeAttribute::populate(anchor_msgs::Anchor &msg) {
    anchor_msgs::ShapeAttribute shape_msg;
    shape_msg.data = this->_data;
    shape_msg.symbols = this->_symbols;
    msg.shape = shape_msg;
  }
  void ShapeAttribute::populate(anchor_msgs::Display &msg) {
    msg.size = _symbols[0];
  }


  float ShapeAttribute::match(const AttributePtr &query_ptr) {

    // Typecast the query pointer
    ShapeAttribute *raw_ptr = dynamic_cast<ShapeAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Jaccard similarity coefficient
    float min = 0.0, max = 0.0;
    min += raw_ptr->_data.x < _data.x ? raw_ptr->_data.x : _data.x;
    min += raw_ptr->_data.y < _data.y ? raw_ptr->_data.y : _data.y;
    min += raw_ptr->_data.z < _data.z ? raw_ptr->_data.z : _data.z;
    max += raw_ptr->_data.x > _data.x ? raw_ptr->_data.x : _data.x;
    max += raw_ptr->_data.y > _data.y ? raw_ptr->_data.y : _data.y;
    max += raw_ptr->_data.z > _data.z ? raw_ptr->_data.z : _data.z;
    if( max > 0.0 ) {
      return min / max;
    }
    return 0.0;
  }

  string ShapeAttribute::toString() {
    return _symbols[0];
  }


  // -------------------------------
  // Caffe attribute class methods
  // ----------------------------------
  CaffeAttribute::CaffeAttribute( const anchor_msgs::CaffeAttribute &msg,
				  AttributeType type ) : AttributeCommon( msg.symbols, type) {
    // Read the data from ROS msg
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( msg.data,
				    sensor_msgs::image_encodings::BGR8 );
      cv_ptr->image.copyTo(this->_data);
    } catch (cv_bridge::Exception& e) {
      throw std::logic_error("[CaffeAttribute::CaffeAttribute]:" + std::string(e.what()) );
    }    
    this->_border = msg.border;
    this->_predictions = vector<double>( msg.predictions.begin(), msg.predictions.end() );
  }

  mongo::Database::Document CaffeAttribute::serialize() {

    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();

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

      // Save (caffe) predictions
      doc.add<double>( "predictions", this->_predictions);

      // Save the contour
      for( auto &point : _border.contour ) {
	mongo::Database::Document p_doc;
	p_doc.add<int>( "x", point.x);
	p_doc.add<int>( "y", point.y);
	doc.append( "border", p_doc);
      }
    }
    catch( const std::exception &e) {
      cout << "[CaffeAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void CaffeAttribute::deserialize(const mongo::Database::Document &doc) {
  
    // Load the image -- from DB
    try {
      std::size_t length = doc.get<std::size_t>("data");
      unsigned char* array = doc.get<unsigned char*>("data");

      // Decompress the image      
      vector<uchar> buff( array, array + length);
      this->_data = cv::imdecode( cv::Mat(buff), CV_LOAD_IMAGE_COLOR);

      // Load (caffe) predictions
      doc.get<double>( "predictions", this->_predictions);

      // Load the contour
      for( mongo::document_iterator ite = doc.begin("border"); ite != doc.end("border"); ++ite) {
	anchor_msgs::Point2d point;
	point.x = ite->get<int>("x");
	point.y = ite->get<int>("y");
	this->_border.contour.push_back(point);	
      }

    }
    catch( const std::exception &e) {
      cout << "[CaffeAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }

  void CaffeAttribute::populate(anchor_msgs::Anchor &msg) {
    anchor_msgs::CaffeAttribute caffe_msg;
    caffe_msg.symbols = this->_symbols;
    for( auto ite = this->_predictions.begin(); ite != this->_predictions.end(); ++ite) {
      caffe_msg.predictions.push_back((float)*ite);
    }
    msg.caffe = caffe_msg;
  }

  void CaffeAttribute::populate(anchor_msgs::Display &msg) {
    msg.border = this->_border;
    float best = 0.0;
    int index = -1;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      if( this->_predictions[i] > best ) {
	best = this->_predictions[i];
	index = i;
      }
    }
    if( index >= 0 ) {
      msg.category = this->_symbols[index];
      msg.prediction = this->_predictions[index];
    }
    
  }

  float CaffeAttribute::match(const AttributePtr &query_ptr) { 

    // Typecast the query pointer
    CaffeAttribute *raw_ptr = dynamic_cast<CaffeAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    float result = 0.0;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      for( uint j = 0; j < raw_ptr->_symbols.size(); j++ ) {
	if( this->_symbols[i] == raw_ptr->_symbols[j] ) {
	  float diff = 1.0 / 
	    exp( abs( this->_predictions[i] - raw_ptr->_predictions[j] ) /
		 ( this->_predictions[i] * raw_ptr->_predictions[j] ) );
	  if (diff > result ) {
	    result = diff;
	  }
	}
      }
    }
    return result;
  }

  bool CaffeAttribute::update(const unique_ptr<AttributeCommon> &query_ptr) {
    
    // Typecast the query pointer
    CaffeAttribute *raw_ptr = dynamic_cast<CaffeAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Updte the contour
    this->_border = raw_ptr->_border;

    // Add non-exisitng symbols
    for( uint i = 0; i < raw_ptr->_symbols.size(); i++ ) {
      if( find( this->_symbols.begin(), this->_symbols.end(), raw_ptr->_symbols[i]) == this->_symbols.end() ) {
	this->_symbols.push_back(raw_ptr->_symbols[i]);
	this->_predictions.push_back(0.0);
      }
    }

    // Update the predictions
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      for( uint j = 0; j < raw_ptr->_symbols.size(); j++ ) {
	if( this->_symbols[i] == raw_ptr->_symbols[j] ) {
	  if( raw_ptr->_predictions[j] > this->_predictions[i] ) {
	    this->_predictions[i] = raw_ptr->_predictions[j];
	  }
	} 
      }
    }

    return true;
  }

  string CaffeAttribute::toString() {
    float best = 0.0;
    int index = -1;
    std::stringstream ss;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      if( this->_predictions[i] > best ) {
	best = this->_predictions[i];
	index = i;
      }
    }
    if( index >= 0 ) {
      ss << this->_symbols[index];
      //ss << this->_symbols[index] << "  (" << this->_predictions[index] << ")";
    }
    return ss.str();
  }

} // namespace anchoring
