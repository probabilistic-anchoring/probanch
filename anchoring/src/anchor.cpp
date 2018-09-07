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
  void Anchor::load(const mongo::Database &db) {
    
    // Load from MongoDB
    try {  
      
      // Load all data as a sub document
      mongo::Database::Document doc = db.get(this->_id); 

      // Read the time
      this->_t = ros::Time( doc.get<double>("t") );

      // Read the unique symbol
      this->_x = doc.get<std::string>("x");
      
      // Read the history
      if( doc.exist("H") ) {
	doc.get<std::string>( "H", this->_history);
      }
      
      // Read all attributes
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
   * Public create function (save the anchor)
   */
  void Anchor::create(mongo::Database &db) {

    // Safely changed the aging variable
    std::lock_guard<std::mutex> lock(this->_mtx);
    if( this->_aging ) { 
      this->_aging = false; // Lives forever (in the database) from now on...
      this->_thread.join();
    }

    // Insert the anchor into database
    try {

      // Generate a unique symbol
      AttributeMap::iterator ite = this->_attributes.find(CAFFE);
      if( ite != this->_attributes.end() ) {
	this->_x = this->generateSymbol( ite->second->toString(), db);
      }
      else {
	this->_x = this->generateSymbol( "unknown", db );
      }

      // Save document (use this _id as identifier)
      mongo::Database::Document doc(this->_id);    

      // Add symbol
      doc.add<std::string>( "x", this->_x);
            
      // Add time
      doc.add<double>( "t", this->_t.toSec());

      // Add the history 
      doc.add<std::string>( "H", this->_x);
      
      // Add all attributes
      for( ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
	mongo::Database::Document subdoc = ite->second->serialize();
        subdoc.add<int>( "type", (int)ite->first);
        doc.append( "attributes", subdoc);
      }
      
      // Commit all changes to database
      db.insert(doc);
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::create]" << e.what() << std::endl;
    }
  }
  
  /**
   * Public maintain function (update the anchor)
   */
  void Anchor::maintain(mongo::Database &db, AttributeMap &attributes, const ros::Time &t ) {

    // Update time
    this->_t = t;

    // The anchor need to be re-acquired within it's life time...
    if( this->_aging ) { 
      this->create(db);    // ...in order to be saved to the database
    }
    else if( !this->compare<AttributeMap>( this->_attributes, attributes ) ) { // Attributes keys does not match 
      this->append(db, attributes);
    }
    else {
      this->update(db, attributes);  // Update existing attributes
    }
  }

  /**
   * Public merge function (combines two anchors)
   */
  void Anchor::merge(mongo::Database &db, AnchorPtr &other) {

    // Merge and update changed attributes
    try {

      // Update the time
      this->_t = ros::Time(other->time());
      db.update<double>( this->_id, "t", this->_t.toSec());      

      // Add and update the history
      if( !other->_history.empty() ) {
	this->_history.insert( this->_history.end(), other->_history.begin(), other->_history.end() );
      }
      this->_history.push_back( other->id() );
      db.update<string>( this->_id, "H", this->_history);            

      // Update the attributes
      if( !this->compare<AttributeMap>( this->_attributes, other->_attributes ) ) { // Map keys are different
	this->append(db, other->_attributes);
      }
      else { // Update existing attributes
	this->update(db, other->_attributes);
      }
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::merge]" << e.what() << std::endl;
    }
  }

  /**
   * Private update functions
   */                                                                                        
  void Anchor::append(mongo::Database &db, AttributeMap &attributes) {

    // Append non-existing attributes and update the database
    try {
    
      // Update time
      db.update<double>( this->_id, "t", this->_t.toSec());
      
      // Add (or move) un-exisiting attributes to this anchor
      std::vector<mongo::Database::Document> docs;
      for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
	if( this->_attributes.find(ite->first) == this->_attributes.end() ) {
	  this->_attributes[ite->first] = std::move(ite->second);
	}
	else {
	  this->_attributes[ite->first]->update(ite->second);
	}
	mongo::Database::Document subdoc = this->_attributes[ite->first]->serialize();
	subdoc.add<int>( "type", (int)ite->first);
	docs.push_back(subdoc);	
      }
      db.update<mongo::Database::Document>( this->_id, "attributes", docs);

      // Check and update the (unique) symbol 
      auto ite = this->_attributes.find(CAFFE);
      if( ite != this->_attributes.end() ) {
	std::string symbol = ite->second->toString();
	if( this->_x.compare( 0, symbol.size(), symbol) != 0 ) {
	  for( auto x : this->_history ) {
	    if( x.compare( 0, symbol.size(), symbol) == 0 ) {
	      this->_x = x;
	      return;
	    }
	  }
	  this->_history.push_back(this->_x);
	  this->_x = this->generateSymbol( symbol, db);
	  db.update<std::string>( this->_id, "x", this->_x);
	  db.update<std::string>( this->_id, "H", this->_history);
	}
      }
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::append]" << e.what() << std::endl;
    }
  }

  void Anchor::update(mongo::Database &db, AttributeMap &attributes) {

    // Update changed attributes
    try {

      // Update time
      db.update<double>( this->_id, "t", this->_t.toSec());

      // Update each attribute
      int idx = 0;
      AttributeMap::iterator ite = this->_attributes.begin();
      for( ; ite != this->_attributes.end(); ++ite, idx++) {
	if( ite->second->update(attributes[ite->first]) ) {
	  std::stringstream ss;
	  mongo::Database::Document doc = ite->second->serialize();
	  doc.add<int>( "type", (int)ite->first);
	  ss << "attributes." << idx;
	  db.update<mongo::Database::Document>( this->_id, ss.str(), doc);	  
	}
      }

      // Check and update the (unique) symbol 
      ite = this->_attributes.find(CAFFE);
      if( ite != this->_attributes.end() ) {
	std::string symbol = ite->second->toString();
	if( this->_x.compare( 0, symbol.size(), symbol) != 0 ) {
	  for( auto x : this->_history ) {
	    if( x.compare( 0, symbol.size(), symbol) == 0 ) {
	      this->_x = x;
	      return;
	    }
	  }
	  this->_history.push_back(this->_x);
	  this->_x = this->generateSymbol( symbol, db);
	  db.update<std::string>( this->_id, "x", this->_x);
	  db.update<std::string>( this->_id, "H", this->_history);
	}
      }  
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::update]" << e.what() << std::endl;
    }
  }

  /**
   * Matching function
   */
  void Anchor::match( const AttributeMap &attributes,
		      MatchMap &result ) {
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
  std::string Anchor::timeStr() {
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
    AttributeMap::iterator ite = this->_attributes.find(POSITION);
    //return this->_x + " " + ite->second->toString();
    return this->_x;
    /*
    std::ostringstream oss;
    for( auto ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {   
      oss << ite->second->toString() << " . ";
    }
    return oss.str();
    */
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

  // ---[ Generate a unique symbol ]---
  std::string Anchor::generateSymbol(const std::string &key, mongo::Database &db) {
    std::stringstream ss;
    try {
      db.set_collection("predicates");
      
      // Get the id (if document exists)
      std::string id = db.get_id( "symbol", key);
      if( !id.empty() ) {
	int counter = db.get<int>( id, "count") + 1;
	db.update<int>( id, "count", counter);
	ss << key << "-" << counter;
      }
      else {
	mongo::Database::Document doc;
	doc.add<std::string>( "symbol", key);
	doc.add<int>( "count", 1);
	db.insert(doc);
	ss << key << "-1";
      }
      db.set_collection("anchors");
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::generateSymbol]" << e.what() << std::endl;
    }
    return ss.str();
  }
  
} // namespace anchoring
