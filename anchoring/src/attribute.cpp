
#include <algorithm>
#include <cassert>

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/assign/list_of.hpp>
#include <boost/filesystem.hpp>

#include <anchoring/attribute.hpp>

namespace anchoring {

  // Set a global threashold for max spikes in distributions
  const double CONST_MAX_TH = 0.65;

  // Used namespace(s)
  using namespace std;

  // --[ Namspace functions ]--  
  void sortAttribute( vector<string>  &symbols,
		      vector<float> &predictions,
		      int n ) {
    for( uint i = 0; i < predictions.size() - 1; i++ ) {
      for( uint j = i + 1; j < predictions.size(); j++ ) {
	if( predictions[i] < predictions[j] ) {
	  swapValues<float>( predictions[i], predictions[j]);
	  swapValues<std::string>( symbols[i], symbols[j]);
	}
      }
    }
    if( n >= 0 && n < (int)symbols.size() ) {
      symbols.erase( symbols.begin() + n, symbols.end());
      predictions.erase( predictions.begin() + n, predictions.end());
    }
  }

  // Mapping between attribute type and string
  AttributeType mapAttributeType(const string &type) {
    if( type == "category" ) return CATEGORY;
    if( type == "color" ) return COLOR;
    if( type == "descriptor" ) return DESCRIPTOR;
    if( type == "position" ) return POSITION;
    return SIZE;
  }

  // Create an attribute (pointer) based on the attribute type  
  AttributePtr createAttribute(AttributeType type) {
    switch(type) {
    case CATEGORY:   return AttributePtr( new CategoryAttribute(type) );
    case COLOR:      return AttributePtr( new ColorAttribute(type) );
    case DESCRIPTOR: return AttributePtr( new DescriptorAttribute(type) );
    case POSITION:   return AttributePtr( new PositionAttribute(type) );
    default:         return AttributePtr( new SizeAttribute(type) );
    };
  }

  
  // ---------------------------------------
  // 0. Common attribute base struct methods
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
    case CATEGORY:   result = "category"; break;
    case COLOR:      result = "color"; break;
    case DESCRIPTOR: result = "descriptor"; break;
    case POSITION:   result = "position"; break;
    case SIZE :      result = "size"; break;
    default:         result = "none"; break;
    }
    return result;
  }


  // -------------------------------
  // 1. Category attribute class methods
  // --------------_--------------------
  CategoryAttribute::CategoryAttribute( const anchor_msgs::CategoryAttribute &msg,
					AttributeType type ) : AttributeCommon( msg.symbols, type) {
    this->_predictions = vector<double>( msg.predictions.begin(), msg.predictions.end() );
    this->_n = 1.0;   // ...counter
  }

  mongo::Database::Document CategoryAttribute::serialize() {

    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();

    // Save (category) predictions
    try {
      doc.add<double>( "predictions", this->_predictions);
      doc.add<double>( "n", this->_n);  // ..including the frequency 
    }
    catch( const std::exception &e) {
      cout << "[CategoryAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void CategoryAttribute::deserialize(const mongo::Database::Document &doc) {
  
    // Load (category) predictions
    try {
      doc.get<double>( "predictions", this->_predictions);
      this->_n = (float)doc.get<double>("n");  // ...including the frequency 
    }
    catch( const std::exception &e) {
      cout << "[CategoryAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }

  void CategoryAttribute::populate(anchor_msgs::Anchor &msg) {
    anchor_msgs::CategoryAttribute category_msg;
    //double max = *std::max_element( this->_predictions.begin(), this->_predictions.end());
    for( uint i = 0; i < this->_predictions.size(); i++ ) {
      //if( (this->_predictions[i] / max) > 0.25 ) {  // ...looking for spikes above max theshold value
      category_msg.symbols.push_back(this->_symbols[i]);
      category_msg.predictions.push_back((float)this->_predictions[i] / this->_n);
      //}
    }
    double max = *std::max_element( this->_predictions.begin(), this->_predictions.end()) / this->_n;
    sortAttribute( category_msg.symbols, category_msg.predictions, (max > 0.95 ? 1 : max > 0.65 ? 2 : 3 ));

    /*
    category_msg.symbols = this->_symbols;
    auto n = this->_N.begin();
    for( auto ite = this->_predictions.begin(); ite != this->_predictions.end(); ++ite, ++n) {
      categoryu_msg.predictions.push_back((float)*ite / *n);
    }
    */
    msg.category = category_msg;
  }

  void CategoryAttribute::populate(anchor_msgs::Display &msg) {
    float best = 0.0;
    int index = -1;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      float prob = this->_predictions[i] / this->_n; 
      if( prob > best ) {
	best = prob;
	index = i;
      }
    }
    if( index >= 0 ) {
      msg.category = this->_symbols[index];
      msg.prediction = this->_predictions[index] / this->_n;
    }
    
  }

  float CategoryAttribute::match(const AttributePtr &query_ptr) { 

    // Typecast the query attribute pointer
    CategoryAttribute *raw_ptr = dynamic_cast<CategoryAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    float result = 0.0;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      for( uint j = 0; j < raw_ptr->_symbols.size(); j++ ) {
	if( this->_symbols[i] == raw_ptr->_symbols[j] && (this->_predictions[i] / this->_n) > 0.05 && raw_ptr->_predictions[j] > 0.05  ) {
	  float diff = 1.0 / 
	    exp( abs( (this->_predictions[i] / this->_n) - raw_ptr->_predictions[j] ) /
		 ( (this->_predictions[i] / this->_n) + raw_ptr->_predictions[j] ) );
	  if (diff > result ) {
	    result = diff;
	  }
	}
      }
    }
    return result;
  }

  bool CategoryAttribute::update(const unique_ptr<AttributeCommon> &new_ptr) {
    
    // Typecast the new attribute pointer
    CategoryAttribute *raw_ptr = dynamic_cast<CategoryAttribute*>(new_ptr.get());
    assert( raw_ptr != nullptr );

    // Increment the counter
    this->_n = this->_n + raw_ptr->_n;
    
    // Summarize the predictions
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      for( uint j = 0; j < raw_ptr->_symbols.size(); j++ ) {
	if( this->_symbols[i] == raw_ptr->_symbols[j] ) {
	  this->_predictions[i] += raw_ptr->_predictions[j];
	}
      }
    }
    return true;
  }

  string CategoryAttribute::toString() {
    float best = 0.0;
    int index = -1;
    std::stringstream ss;
    for( uint i = 0; i < this->_symbols.size(); i++ ) {
      float prob = this->_predictions[i] / this->_n;
      if( prob > best ) {
	best = prob;
	index = i;
      }
    }
    if( index >= 0 ) {
      ss << this->_symbols[index];
    }
    return ss.str();
  }


  // --------------------------------
  // 2. Color attribute methods
  // ------------------------------------  
  ColorAttribute::ColorAttribute( const anchor_msgs::ColorAttribute &msg, 
				  AttributeType type ) : AttributeCommon( msg.symbols, type) {
    // Initilize counter
    this->_n = 1.0;  
    
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

      // Save the counter
      doc.add<double>( "n", this->_n);
      
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
      this->_data = cv::Mat( 1, array.size(), CV_32FC1);
      for( int i = 0; i < array.size(); i++) {
	this->_data.at<float>( 0, i) = (float)array[i];
      } 
      
      // Load the counter
      this->_n = (float)doc.get<double>("n");
      
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
    double max = *std::max_element( this->_predictions.begin(), this->_predictions.end());
    for( uint i = 0; i < this->_predictions.size(); i++ ) {
      if( (this->_predictions[i] / max) > CONST_MAX_TH ) {  // ...looking for spikes above max theshold value 
	color_msg.symbols.push_back(this->_symbols[i]);
	color_msg.predictions.push_back((float)this->_predictions[i] / this->_n);	
      }
    }
    sortAttribute( color_msg.symbols, color_msg.predictions, 5);
    msg.color = color_msg;
  }
  void ColorAttribute::populate(anchor_msgs::Display &msg) {
    double max = *std::max_element( this->_predictions.begin(), this->_predictions.end());
    for( uint i = 0; i < this->_predictions.size(); i++ ) {
      if( (this->_predictions[i] / max) > CONST_MAX_TH ) {  // ...looking for spikes above max theshold value 
	msg.colors.push_back(this->_symbols[i]);
      }
    }
    // msg.colors = this->_symbols;
  }
  
  float ColorAttribute::match(const AttributePtr &query_ptr) {
    
    // Typecast the query attribute pointer
    ColorAttribute *raw_ptr = dynamic_cast<ColorAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    float dist = (1.0 + cv::compareHist( raw_ptr->_data, (this->_data / this->_n), CV_COMP_CORREL)) / 2.0; // CV_COMP_CORREL | CV_COMP_INTERSECT | CV_COMP_BHATTACHARYYA
    //std::cout << "Color dist: " << dist << std::endl;
    return dist;
  }

  bool ColorAttribute::update(const AttributePtr &new_ptr) {

    // Typecast the new attribute pointer
    ColorAttribute *raw_ptr = dynamic_cast<ColorAttribute*>(new_ptr.get());
    assert( raw_ptr != nullptr );

    // Summarize histograms
    this->_data = this->_data + raw_ptr->_data;
    
    // Increment the counter
    this->_n = this->_n + raw_ptr->_n;

    // Summarize the predictions
    for( uint i = 0; i < this->_predictions.size(); i++ ) {
      this->_predictions[i] += raw_ptr->_predictions[i];
    }

    return true;
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
  // 3. Descriptor attribute methods
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
      if( length > 0 ) {
	doc.add<unsigned char*>( "data", this->_data.data, length);
      }
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
      if( rows > 0 ) {
	unsigned char* array = doc.get<unsigned char*>( "data");
	this->_data = cv::Mat( rows, cols, CV_8U, array);
      }
    }
    catch( const std::exception &e) {
      cout << "[DescriptorAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }


  // ----------------------------------
  // 4. Position attribute methods
  // ---------------------------------------
  PositionAttribute::PositionAttribute( const anchor_msgs::PositionAttribute &msg,
					AttributeType type ) : AttributeCommon( msg.symbols, type) {
    // Add the data (including a timestamp)
    this->_array.push_back(msg.data);
  }


  PositionAttribute::PositionAttribute( const vector<geometry_msgs::PoseStamped> &array, 
					const vector<string> &symbols,
					AttributeType type ) : AttributeCommon( symbols, type) {
      // Add the data (including a timestamp)
    for( auto &pos : array ) {
      this->_array.push_back(pos);
    }
  }

  
  PositionAttribute::PositionAttribute( const AttributePtr &ptr,
					AttributeType type) : AttributeCommon(type) {

    // Typecast the query pointer
    PositionAttribute *raw_ptr = dynamic_cast<PositionAttribute*>(ptr.get());
    assert( raw_ptr != nullptr );

    // Add the data (including a timestamp)
    for( auto &pos : raw_ptr->_array ) {
      this->_array.push_back(pos);
    }
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

  // Overleaded functions for populating ROS messages
  void PositionAttribute::populate(anchor_msgs::Anchor &msg) {
    //anchor_msgs::PositionAttribute poistion;
    if( !this->_array.empty() ) {
      for( const auto &pos : this->_array ) {
        msg.position.data.pose.position.x += pos.pose.position.x;
        msg.position.data.pose.position.y += pos.pose.position.y;
        msg.position.data.pose.position.z += pos.pose.position.z;
        msg.position.data.pose.orientation.x += pos.pose.orientation.x;
        msg.position.data.pose.orientation.y += pos.pose.orientation.y;
        msg.position.data.pose.orientation.z += pos.pose.orientation.z;
        msg.position.data.pose.orientation.w += pos.pose.orientation.w;
      }
      msg.position.data.pose.position.x /= (int)this->_array.size();
      msg.position.data.pose.position.y /= (int)this->_array.size();
      msg.position.data.pose.position.z /= (int)this->_array.size();
      msg.position.data.pose.orientation.x /= (int)this->_array.size();
      msg.position.data.pose.orientation.y /= (int)this->_array.size();
      msg.position.data.pose.orientation.z /= (int)this->_array.size();
      msg.position.data.pose.orientation.w /= (int)this->_array.size();
    }
    //msg.position = poistion;
  }
  void PositionAttribute::populate(anchor_msgs::Display &msg) {
    if( !this->_array.empty() ) {
      for( const auto &pos : this->_array ) {
	msg.pos.pose.position.x += pos.pose.position.x;
	msg.pos.pose.position.y += pos.pose.position.y;
	msg.pos.pose.position.z += pos.pose.position.z;
      }
      msg.pos.pose.position.x /= (int)this->_array.size();
      msg.pos.pose.position.y /= (int)this->_array.size();
      msg.pos.pose.position.z /= (int)this->_array.size();
    }
  }
  
  float PositionAttribute::match(const AttributePtr &query_ptr) {

    // Typecast the query attribute pointer
    PositionAttribute *raw_ptr = dynamic_cast<PositionAttribute*>(query_ptr.get());
    assert( raw_ptr != nullptr );

    // The normalized L2 distance
    double _x, _y, _z, _d, _dist = -1.0;
    for( uint i = 0; i < this->_array.size(); i++ ) {
      geometry_msgs::PoseStamped train = this->_array[i];
      for( uint j = 0; j < raw_ptr->_array.size(); j++ ) {      
	geometry_msgs::PoseStamped query = raw_ptr->_array[j];
	_x = query.pose.position.x - train.pose.position.x;
	_y = query.pose.position.y - train.pose.position.y;
	_z = query.pose.position.z - train.pose.position.z;
	_d  = sqrt( _x*_x + _y*_y + _z*_z );
	if( _dist < 0.0 || _d < _dist ) {
	  _dist = _d;
	}
      }
    }
    if( _dist >= 0.0 ) {
      return 1.0 / exp(_dist);
    }
    return 0.0;
  }

  bool PositionAttribute::update(const unique_ptr<AttributeCommon> &new_ptr) {
    
    // Typecast the new attribute pointer
    PositionAttribute *raw_ptr = dynamic_cast<PositionAttribute*>(new_ptr.get());
    assert( raw_ptr != nullptr );

    // Update the location
    this->_array.clear();
    for( auto &pos : raw_ptr->_array ) {
      this->_array.push_back(pos);
    }

    /*
    // Update the location
    if( this->match(new_ptr) < 0.01 ) { // ...not moved more than 1cm
      this->_array.back().header.stamp = raw_ptr->_array.back().header.stamp;
      // this->_array.back() = raw_ptr->_array.front();
      }
    else {
      // Append the location (including a timestamp)
      this->_array.push_back(raw_ptr->_array.back());
    }
    */
    
    /*
    // Append the symbol (if there exists an symbol)
    if( !raw_ptr->_symbols.empty() ) {
      this->_symbols.back() = raw_ptr->_symbols.front();
      this->_symbols.push_back(raw_ptr->_symbols.front());
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
  // 5. Size attribute class methods
  // -------------------------------------
  SizeAttribute::SizeAttribute( const anchor_msgs::SizeAttribute &msg,
				AttributeType type ) : AttributeCommon( msg.symbols, type) {
    this->_data = msg.data;
  }

  mongo::Database::Document SizeAttribute::serialize() {
  
    // Save the symbol(s)
    mongo::Database::Document doc = AttributeCommon::serialize();

    // Save the shape
    try {
      std::vector<double> size;
      size.push_back(this->_data.x);
      size.push_back(this->_data.y);
      size.push_back(this->_data.z);
      doc.add<double>( "data", size);
    }
    catch( const std::exception &e) {
      cout << "[SizeAttribute::serialize]" << e.what() << endl;
    }
    return doc;
  }

  void SizeAttribute::deserialize(const mongo::Database::Document &doc) {

    // Load the shape
    try {
      vector<double> size;
      doc.get<double>( "data", size);
      this->_data.x = size[0];
      this->_data.y = size[1];
      this->_data.z = size[2];
    }
    catch( const std::exception &e) {
      cout << "[SizeAttribute::deserialize]" << e.what() << endl;
    }

    // Load the symbol(s) 
    AttributeCommon::deserialize(doc);
  }

  void SizeAttribute::populate(anchor_msgs::Anchor &msg) {
    anchor_msgs::SizeAttribute size_msg;
    size_msg.data = this->_data;
    size_msg.symbols = this->_symbols;
    msg.size = size_msg;
  }
  void SizeAttribute::populate(anchor_msgs::Display &msg) {
    msg.size = _symbols[0];
  }

  float SizeAttribute::match(const AttributePtr &query_ptr) {

    // Typecast the query attribute pointer
    SizeAttribute *raw_ptr = dynamic_cast<SizeAttribute*>(query_ptr.get());
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
  
  bool SizeAttribute::update(const unique_ptr<AttributeCommon> &new_ptr) {

    // Typecast the new atribute pointer
    SizeAttribute* raw_ptr = dynamic_cast<SizeAttribute*>(new_ptr.get());
    assert( raw_ptr != nullptr );

    this->_data = raw_ptr->_data;
  }
  
  string SizeAttribute::toString() {
    return _symbols[0];
  }
  
  
} // namespace anchoring
