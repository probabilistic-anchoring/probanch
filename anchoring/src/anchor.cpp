/* ------------------------------------------ 


   ------------------------------------------ */
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <anchoring/anchor.hpp>
#include <anchoring/database.hpp>

namespace anchoring {

  using namespace std;

  // Constructor
  Anchor::Anchor(AttributeMap &attributes, const ros::Time &t) {

    // Assign (or move) the attributes
    for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
      this->_attributes[ite->first] = std::move(ite->second);
    }
    
    // Generate an id  
    this->_id = MongoDatabase::generateId();
    
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
  void Anchor::load(const MongoDatabase &db) {
    
    // Load from MongoDB
    try {  
      
      // Read the time
      this->_t = ros::Time( db.get<double>("t") );

      // Read all attributes
      MongoDatabase::Subdoc sub;
      db.get( "attributes", sub);
      while( !sub.empty() ) {
      
	// Load the type...
	AttributeType type = (AttributeType)sub.get<int>("type");

	// Load the attribute based on the type
	switch(type) {
	case DESCRIPTOR:
	  this->_attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(type) );
	  break;
	case COLOR:
	  this->_attributes[COLOR] = AttributePtr( new ColorAttribute(type) );
	  break;
	case LOCATION:
	  this->_attributes[LOCATION] = AttributePtr( new LocationAttribute(type) );
	  break;
	case SHAPE:
	  this->_attributes[SHAPE] = AttributePtr( new ShapeAttribute(type) );
	  break;
	case CAFFE:
	  this->_attributes[CAFFE] = AttributePtr( new CaffeAttribute(type) );
	  break;
	};
	this->_attributes[type]->deserialize(sub); // Deserialize the attribute data
	sub.pop(); // Get next
      } 

    }
    catch( DBException &e ) {
      std::cout << "[anchor::load] MondoDB: " << e.what() << std::endl;
    }
  }
  
  /**
   * Private save function 
   */
  void Anchor::save(MongoDatabase &db) {
    
    // Insert the anchor into database
    try {

      // Prepare to save (use this _id as identifier)
      db.prepareInsert(this->_id);

      // Add time
      db.add<double>( "t", this->_t.toSec());

      // Add all attributes
      MongoDatabase::Subdoc sub;
      AttributeMap::iterator ite;
      for( ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
        sub.add<int>( "type", (int)ite->first);
        ite->second->serialize(sub);
        sub.push();
      }
      db.add( "attributes", sub);
      
      // Commit all changes to database
      db.insert();
    }
    catch( DBException &e ) {
      std::cout << "[anchor::save] MondoDB: " << e.what() << std::endl;
    }
  }
  
  /**
   * Public update function 
   */
  void Anchor::update(MongoDatabase &db, AttributeMap &attributes, const ros::Time &t, bool append ) {

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

	// Add alla attributes
	MongoDatabase::Subdoc sub;
	for( auto ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
	  sub.add<int>( "type", (int)ite->first);
	  ite->second->serialize(sub);
	  sub.push();
	}
	db.update( this->_id, "attributes", sub);
      }
      catch( DBException &e ) {
	cout << "[anchor::update] MondoDB: " << e.what() << endl;
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
