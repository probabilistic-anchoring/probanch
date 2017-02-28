/* ------------------------------------------ 


   ------------------------------------------ */
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <anchoring/anchor.hpp>

namespace anchoring {

  using namespace std;

  // Constructor
  Anchor::Anchor(AttributeMap &attributes, const ros::Time &t) {

    // Assign (or move) the attributes
    for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
      this->_attributes[ite->first] = std::move(ite->second);
    }
    
    // Generate an id  
    this->_id = mongo::Database::generate_id();
    
    // Set time
    this->_t = t;

    // Set status variables
    this->_aging = true;
    this->_thread = std::thread( &Anchor::decreaseAnchor, this, 250);
  }

  // Destructor
  Anchor::~Anchor() {
    if( this->_thread.joinable() ) {
      this->_thread.join();
    }
    this->_attributes.clear();
  }  

  /**
   * Public load (override of base class)
   */
  void Anchor::load(const mongo::Database::Document &doc) {
    
    // Load from MongoDB
    try {  
      
      // Read the time
      this->_t = ros::Time( doc.get<double>("t") );

      // Read all attributes
      //std::size_t size = db.get<std::size_t>("attributes");
      //for( std::size_t i = 0; i < size; i++) {
      for( mongo::document_iterator ite = doc.begin("attributes"); ite != doc.end("attributes"); ++ite) {

	// Load the type...
	AttributeType type = (AttributeType)ite->get<int>("type");

	// Load the attribute based on the type
	switch(type) {
	case DESCRIPTOR:
	  this->_attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(type) );
	  break;
	case COLOR:
	  this->_attributes[COLOR] = AttributePtr( new ColorAttribute(type) );
	  break;
	case POSITION:
	  this->_attributes[POSITION] = AttributePtr( new PositionAttribute(type) );
	  break;
	case SHAPE:
	  this->_attributes[SHAPE] = AttributePtr( new ShapeAttribute(type) );
	  break;
	case CAFFE:
	  this->_attributes[CAFFE] = AttributePtr( new CaffeAttribute(type) );
	  break;
	};

	// Deserialize the attribute data
	this->_attributes[type]->deserialize(*ite); 
      } 
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::load]" << e.what() << std::endl;
    }
  }
  
  /**
   * Private save function 
   */
  void Anchor::save(mongo::Database &db) {
    
    // Insert the anchor into database
    try {

      // Save document (use this _id as identifier)
      mongo::Database::Document doc(this->_id);     
      
      // Add time
      doc.add<double>( "t", this->_t.toSec());
      
      // Add all attributes
      AttributeMap::iterator ite;
      for( ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
	mongo::Database::Document subdoc = ite->second->serialize();
        subdoc.add<int>( "type", (int)ite->first);
        doc.append( "attributes", subdoc);
      }
      
      // Commit all changes to database
      db.insert(doc);
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::save]" << e.what() << std::endl;
    }
  }
  
  /**
   * Public update function 
   */
  void Anchor::update(mongo::Database &db, AttributeMap &attributes, const ros::Time &t, bool append ) {

    // Update the time and append data (if the anchor has moved)
    if( append ) {
      this->append(attributes, t);
    }
    else {
      this->_t = t;
    }    

    // The anchor need to be re-acquired within it's life time...
    if( this->_thread.joinable() ) { 
      this->_mtx.lock();
      this->_aging = false; // Forever young from now on....
      this->_mtx.unlock();
      this->_thread.detach();

      // ...in order to be saved to the database
      this->save(db);
    }
    else {

      // Add (or move) un-exisiting attributes to this anchor
      for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
	if( this->_attributes.find(ite->first) == this->_attributes.end() ) {
	  this->_attributes[ite->first] = std::move(ite->second);
	}
	else if( !append ) {
	  this->_attributes[ite->first]->update(ite->second);
	}
      }

      // Update the database
      try {

	// Add time
	db.update<double>( this->_id, "t", this->_t.toSec());

	// Add all attributes
	std::vector<mongo::Database::Document> docs;
	AttributeMap::iterator ite;
	for( ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
	  mongo::Database::Document subdoc = ite->second->serialize();
	  subdoc.add<int>( "type", (int)ite->first);
	  docs.push_back(subdoc);
	}
	db.update<mongo::Database::Document>( this->_id, "attributes", docs);
      }
      catch( const std::exception &e ) {
	std::cout << "[Anchor::update]" << e.what() << std::endl;
      }
    }
  }

  /**
   * Public append function
   */                                                                                        
  void Anchor::append(AttributeMap &attributes, const ros::Time &t) {

    // Update time
    this->_t = t;

    // Append new data to existing attributes (but no database update) 
    for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
      if( this->_attributes.find(ite->first) != this->_attributes.end() ) {
	this->_attributes[ite->first]->append(ite->second);
      }
    }
  }


  /**
   * Matching function
   */
  void Anchor::match( const AttributeMap &attributes,
		      MatchMap &result ) {
    //AttributeMap::const_iterator ite;
    for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
      if( this->_attributes.find(ite->first) != this->_attributes.end() ) {
	std::pair<AttributeType, float> rate;
	rate.first = ite->first;	
	rate.second = this->_attributes[ite->first]->match(ite->second);
	result.insert(rate);
      }
    }
  } 

  /**
   * Get/set functions + private life-span function
   */

  /*
  // Get anc OpenCV "image"
  cv::Mat Anchor::get(AttributeType type) {
    AttributeMap::iterator ite = this->_attributes.find(type);
    if( ite != this->_attributes.end() ) {
      if( type == DESCRIPTOR ) {
	return dynamic_cast<DescriptorAttribute*>(ite->second.get())->_data;
      }
      else if( type == COLOR ) {
	return dynamic_cast<ColorAttribute*>(ite->second.get())->_data;
      }
      else if( type == CAFFE ) {
	return dynamic_cast<ImageAttribute*>(ite->second.get())->_data;
      }
    }
    return cv::Mat();
  }
  */

  // Set the symbol(s) of an attribute of an anchor
  void Anchor::setSymbols( AttributeType type, const vector<string> &symbols) {
    AttributeMap::iterator ite = this->_attributes.find(type);
    if( ite != this->_attributes.end() ) {
      ite->second->_symbols = symbols;
    }
  }
  
  // Get the symbol(s) for an attribute of an anchor
  vector<string> Anchor::getSymbols(AttributeType type) {
    AttributeMap::iterator ite = this->_attributes.find(type);
    if( ite != this->_attributes.end() ) {
      return ite->second->_symbols;
    }
    return vector<string>();
  }

  // Get time string
  std::string Anchor::getTimeStr() {
    boost::posix_time::ptime boost_t = this->_t.toBoost();
    return boost::posix_time::to_simple_string(boost_t);
  }
  
  // Thread function for decreasing the life of the anchor
  void Anchor::decreaseAnchor(int life) {
    while( this->_aging && life > 0  ) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      life--;
    }
    
    // Dead anchor - release this object
    if( this->_aging ) {
      this->_attributes.clear();
    }
  }

  std::string Anchor::toString() { 
    std::ostringstream oss;
    for( auto ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {   
      oss << ite->second->toString() << " . ";
    }
    return oss.str();
  }
    
  /*
  anchor_msgs::Snapshot Anchor::getSnapshot() {
    anchor_msgs::Snapshot msg;
    for( auto ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
      msg.id = this->_id;
      ite->second->populate(msg);
    }
    return msg;
  }
  */
  
} // namespace anchoring
