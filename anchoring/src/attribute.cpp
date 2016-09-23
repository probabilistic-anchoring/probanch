
#include <ros/package.h>
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

  /**
   * Common attribute base struct methods
   */
  void AttributeCommon::serialize(MongoDatabase::Subdoc &db_sub) {

    // Save the symbols
    try {
      db_sub.add<string>( "symbols", this->_symbols);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }
  }

  void AttributeCommon::deserialize(const MongoDatabase::Subdoc &db_sub) {

    // Load the symbols
    try {
      db_sub.get<string>( "symbols", this->_symbols);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }
  }

  // Symbol mapping of the attribute type
  string AttributeCommon::getTypeStr() {
    string result;
    switch(this->_type) {
    case DESCRIPTOR: result = "descriptor"; break;
    case COLOR:      result = "color"; break;
    case SHAPE:      result = "shape"; break;
    case LOCATION:   result = "location"; break;
    default:         result = "image"; break;
    }
    return result;
  }


  /**
   * Color attribute struct methods
   */  
  ColorAttribute::ColorAttribute( const vector<uint16_t> &data,
				  const vector<string> &symbols,
				  AttributeType type ) 
    : AttributeCommon( symbols, type) {
    this->_total = 0;
    for( uint i = 0; i < data.size(); i++) { 
      this->_total += data[i];
    }
    this->_data = cv::Mat( 1, data.size(), CV_32F);
    for( uint i = 0; i < data.size(); i++) { 
      this->_data.at<float>( 0, i) = data[i] / (float)this->_total;
    }  
  }

  void ColorAttribute::serialize(MongoDatabase::Subdoc &db_sub) {
  
    // Save the color
    try {
      std::vector<double> array;      
      for( int i = 0; i < this->_data.cols; i++) {
	array.push_back(this->_data.at<float>( 0, i));
      }
      db_sub.add<double>( "data", array);
      db_sub.add<int>( "samples", this->_total);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }
    
    // Save the symbol(s)
    AttributeCommon::serialize(db_sub);
  }

  void ColorAttribute::deserialize(const MongoDatabase::Subdoc &db_sub) {

    // Load the color 
    try {
      this->_total = db_sub.get<int>("samples");
      std::vector<double> array;
      db_sub.get<double>( "data", array);
      this->_data = cv::Mat( 1, array.size(), CV_32F);
      for( uint i = 0; i < array.size(); i++) {
	this->_data.at<float>( 0, i) = (float)array[i]; 
      }
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(db_sub);
  }

  void ColorAttribute::populate(anchor_msgs::Snapshot &msg) {
    msg.color = _symbols;
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


  /**
   * Descriptor attribute class methods
   */
  void DescriptorAttribute::serialize(MongoDatabase::Subdoc &db_sub) {
  
    // Save the descriptor
    try {
      db_sub.add<int>( "rows", (int)this->_data.rows);
      db_sub.add<int>( "cols", (int)this->_data.cols);

      uint length = this->_data.rows * this->_data.cols;
      unsigned char* array = new unsigned char[length];
      for( int i = 0; i < this->_data.rows; i++) { 
	unsigned char* ptr = this->_data.ptr<unsigned char>(i);
	for( int j = 0; j < this->_data.cols; j++) {
	  array[i * this->_data.cols + j] = ptr[j];
	}      
      }
      db_sub.addBinary( "data", array, length);
      delete array;
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }

    // Save the symbol(s)
    AttributeCommon::serialize(db_sub);
  }

  void DescriptorAttribute::deserialize(const MongoDatabase::Subdoc &db_sub) {

    // Load the descriptor
    try {
      int rows = db_sub.get<int>("rows");
      int cols = db_sub.get<int>("cols");
      unsigned char* array = new unsigned char[rows * cols];
      db_sub.getBinary( "data", array);
      cv::Mat descriptor( rows, cols, CV_8U, array);
      descriptor.copyTo(this->_data);
      delete array;
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }    

    // Load the symbol(s) 
    AttributeCommon::deserialize(db_sub);
  }


  /**
   * Location attribute class methods
   */
  void LocationAttribute::serialize(MongoDatabase::Subdoc &db_sub) {
  
    // Save the location
    try {
      //vector<string>::iterator ite_s = this->_symbols.begin();
      vector<geometry_msgs::PoseStamped>::iterator ite_d = this->_array.begin();
      MongoDatabase::Subdoc sub;
      for( ; ite_d != this->_array.end(); ++ite_d /*, ++ite_s*/ ) {  

	// Add symbol
	//sub.add<string>( "symbol", *ite_s);

	// Add time
	sub.add<double>( "t", ite_d->header.stamp.toSec());

	// Add position
	std::vector<double> position; 
	position.push_back(ite_d->pose.position.x);
	position.push_back(ite_d->pose.position.y);
	position.push_back(ite_d->pose.position.z);
	sub.add<double>( "data.position", position);

	// Add orientation
	std::vector<double> orientation;
	orientation.push_back(ite_d->pose.orientation.x);
	orientation.push_back(ite_d->pose.orientation.y);
	orientation.push_back(ite_d->pose.orientation.z);
	orientation.push_back(ite_d->pose.orientation.w);
	sub.add<double>( "data.orientation", orientation);

	sub.push();
      }
      db_sub.add( "array", sub);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }
  }

  void LocationAttribute::deserialize(const MongoDatabase::Subdoc &db_sub) {
    
    // Load the location
    try {
      
      // Read a series of location data
      MongoDatabase::Subdoc sub;
      db_sub.get( "array", sub);
      while( !sub.empty() ) {
	geometry_msgs::PoseStamped data;

	// Read the time
	data.header.stamp = ros::Time(sub.get<double>("t"));	

	// Read the symbol
	//string symbol = sub.get<string>("symbol");
	//this->_symbols.push_back(symbol);

	// Read the data
	std::vector<double> position;
	sub.get<double>( "data.position", position);
	data.pose.position.x = position[0];
	data.pose.position.y = position[1];
	data.pose.position.z = position[2];
	
	std::vector<double> orientation;
	sub.get<double>( "data.orientation", orientation);
	data.pose.orientation.x = orientation[0];
	data.pose.orientation.y = orientation[1];
	data.pose.orientation.z = orientation[2];
	data.pose.orientation.w = orientation[3];
	this->_array.push_back(data);

	sub.pop(); // Get next
      }
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }
  }

  void LocationAttribute::populate(anchor_msgs::Snapshot &msg) {
    msg.center = this->_array.back();
  }
  

  float LocationAttribute::match(const AttributePtr &query_ptr) {

    // Typecast the query pointer
    LocationAttribute *raw_ptr = dynamic_cast<LocationAttribute*>(query_ptr.get());
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

  void LocationAttribute::append(const unique_ptr<AttributeCommon> &query_ptr) {
    
    // Typecast the query pointer
    LocationAttribute *raw_ptr = dynamic_cast<LocationAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Append the location (including a timestamp)
    this->_array.push_back(raw_ptr->_array.front());

    // Append the symbol (if there exists an symbol)
    if( !raw_ptr->_symbols.empty() ) {
      this->_symbols.push_back(raw_ptr->_symbols.front());
    }  
  }

  void LocationAttribute::update(const unique_ptr<AttributeCommon> &query_ptr) {
    
    // Typecast the query pointer
    LocationAttribute *raw_ptr = dynamic_cast<LocationAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // Update the location
    this->_array.back().header.stamp = raw_ptr->_array.front().header.stamp;
    /*
    this->_array.back() = raw_ptr->_array.front();
    if( !raw_ptr->_symbols.empty() ) {
      this->_symbols.back() = raw_ptr->_symbols.front();
    }
    */
  }

  string LocationAttribute::toString() {
    std::stringstream ss;
    ss << "@ {";
    ss << " x = " << this->_array.back().pose.position.x;
    ss << ", y = " << this->_array.back().pose.position.y;
    ss << ", z = " << this->_array.back().pose.position.z;
    ss << "}";
    return ss.str();
  }

  
  /**
   * Shape attribute class methods
   */
  void ShapeAttribute::serialize(MongoDatabase::Subdoc &db_sub) {
  
    // Save the shape
    try {
      std::vector<double> shape;
      shape.push_back(this->_data.x);
      shape.push_back(this->_data.y);
      shape.push_back(this->_data.z);
      db_sub.add<double>( "data", shape);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }

    // Save the symbol(s)
    AttributeCommon::serialize(db_sub);
  }

  void ShapeAttribute::deserialize(const MongoDatabase::Subdoc &db_sub) {

    // Load the shape
    try {
      std::vector<double> shape;
      db_sub.get<double>( "data", shape);
      this->_data.x = shape[0];
      this->_data.y = shape[1];
      this->_data.z = shape[2];
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;;
    }    

    // Load the symbol(s) 
    AttributeCommon::deserialize(db_sub);
  }

  void ShapeAttribute::populate(anchor_msgs::Snapshot &msg) {
    msg.bounding_box = this->_data;
    msg.size = _symbols[0];
    msg.shape = _symbols[1];
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


  /**
   * Caffe attribute class methods
   */
  void CaffeAttribute::serialize(MongoDatabase::Subdoc &db_sub) {

    // Compress the image  
    vector<uchar> buff;
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);
    cv::imencode( ".jpg", this->_data, buff, compression_params);

    // Save the image (to DB)
    try {
      int length = (int)buff.size();
      db_sub.add<int>( "length", length);
      unsigned char* array = new unsigned char[length];
      for( uint i = 0; i < buff.size(); i++) { 
	array[i] = buff[i];      
      }
      db_sub.addBinary( "data", array, length);
      delete array;

      // Save (caffe) predictions
      db_sub.add<double>( "predictions", this->_predictions);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }

    // Save the symbol(s)
    AttributeCommon::serialize(db_sub);
  }

  void CaffeAttribute::deserialize(const MongoDatabase::Subdoc &db_sub) {
  
    // Load the image -- from DB
    try {
      int length = db_sub.get<int>("length");
      unsigned char* array = new unsigned char[length];
      db_sub.getBinary( "data", array);

      // Decompress the image      
      vector<uchar> buff( array, array + length);
      this->_data = cv::imdecode( cv::Mat(buff), CV_LOAD_IMAGE_COLOR);
      delete array;

      // Load (caffe) predictions
      db_sub.get<double>( "predictions", this->_predictions);
    }
    catch( DBException &e ) {
      cout << "[attribute] MondoDB: " << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(db_sub);
  }

  void CaffeAttribute::populate(anchor_msgs::Snapshot &msg) {
    float best = 0.0;
    int index = -1;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      if( this->_predictions[i] > best ) {
	best = this->_predictions[i];
	index = i;
      }
    }
    if( index >= 0 ) {
      msg.object = this->_symbols[index];
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

  void CaffeAttribute::update(const unique_ptr<AttributeCommon> &query_ptr) {
    
    // Typecast the query pointer
    CaffeAttribute *raw_ptr = dynamic_cast<CaffeAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

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
      ss << this->_symbols[index] << "  (" << this->_predictions[index] << ")";
    }
    return ss.str();
  }

} // namespace anchoring
