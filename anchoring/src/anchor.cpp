/* ------------------------------------------ 


   ------------------------------------------ */
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <anchoring/anchor.hpp>

namespace anchoring {

  using namespace std;

  // Constructor
  Anchor::Anchor(const ros::Time &t, AttributeMap &attributes, PerceptMap &percepts) {

    // Assign (or move) the attributes
    for( auto ite = attributes.begin(); ite != attributes.end(); ++ite) {
      this->_attributes[ite->first] = std::move(ite->second);
    }

    // Assign (or move) the perecpts
    for( auto ite = percepts.begin(); ite != percepts.end(); ++ite) {
      this->_percepts[ite->first] = std::move(ite->second);
    }
    
    // Generate an id  
    this->_id = mongo::Database::generate_id();
    
    // Set time
    this->_t = t;

    // Set status variables
    this->_persistent = false;
    this->_thread = std::thread( &Anchor::fading, this, 100);
  }

  // Destructor
  Anchor::~Anchor() {
    if( this->_thread.joinable() ) {
      this->_thread.join();
    }
    this->_attributes.clear();
  }  

  /**
   * Public load function (retreive the anchor from the database)
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

      // Read all percepts
      for( mongo::document_iterator ite = doc.begin("percepts"); ite != doc.end("percepts"); ++ite) {

	// Load the type... 
	PerceptType type = mapPerceptType(ite->get<std::string>("type"));

	// Load the percepts based on the type
	this->_percepts[type] = createPercept(type);

	// Deserialize the perceptual data
	this->_percepts[type]->deserialize(*ite); 
      } 
      
      // Read all attributes
      for( mongo::document_iterator ite = doc.begin("attributes"); ite != doc.end("attributes"); ++ite) {

	// Load the type...
	AttributeType type = mapAttributeType(ite->get<std::string>("type"));
	//AttributeType type = (AttributeType)ite->get<int>("type");
	
	// Load the attribute based on the type
	this->_attributes[type] = createAttribute(type);

	// Deserialize the attribute data
	this->_attributes[type]->deserialize(*ite); 
      } 
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::load]" << e.what() << std::endl;
    }
  }
  
  /**
   * Public save function (stores the anchor in the database)
   */
  void Anchor::save(mongo::Database &db) {

    // Safely changed the persistent variable
    std::lock_guard<std::mutex> lock(this->_mtx);
    if( !this->_persistent ) { 
      this->_persistent = true; // Lives forever (in the database) from now on...
      if( this->_thread.joinable() ) {
	this->_thread.join();
      }
    }

    // Insert the anchor into database
    try {

      // Generate a unique symbol
      if( this->_attributes.find(CATEGORY) != this->_attributes.end() ) {
	auto it = this->_attributes.find(CATEGORY);
	this->_x = this->generateSymbol( it->second->toString(), db);
      }
      else {
	this->_x = this->generateSymbol( "unknown", db );
      }

      // Save document (use this _id as identifier)
      mongo::Database::Document doc(this->_id);    

      // Add symbol
      doc.add<std::string>( "x", this->_x);

      // Add the history 
      doc.add<std::string>( "H", this->_x);

      // Add time
      doc.add<double>( "t", this->_t.toSec());

      // Add all attributes
      for( auto it = this->_attributes.begin(); it != this->_attributes.end(); ++it) {
	mongo::Database::Document subdoc = it->second->serialize();
        subdoc.add<std::string>( "type", it->second->getTypeStr());
        doc.append( "attributes", subdoc);
      }

      // Add all percepts
      for( auto it = this->_percepts.begin(); it != this->_percepts.end(); ++it) {
	mongo::Database::Document subdoc = it->second->serialize();
        subdoc.add<std::string>( "type", it->second->getTypeStr());
        doc.append( "percepts", subdoc);
      }

      // Commit all changes to database
      db.insert(doc);
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::save]" << e.what() << std::endl;
    }
  }
  
  /**
   * Public update function(s)
   */
  void Anchor::update(mongo::Database &db, const ros::Time &t, AttributeMap &attributes, PerceptMap &percepts ) {

    // Update the percepts of this anchor (or move un-exisiting percepts to this anchor) 
    for( auto it = percepts.begin(); it != percepts.end(); ++it) {
      if( this->_percepts.find(it->first) == this->_percepts.end() ) {
	this->_percepts[it->first] = std::move(it->second);
      }
      else {
	this->_percepts[it->first]->update(it->second);
      }
    }

    // Make use of function overloading...
    this->update(db, t, attributes);
  }
  
  void Anchor::update(mongo::Database &db, const ros::Time &t, AttributeMap &attributes ) {

    // Update time
    this->_t = t;

    // Update attributes of this anchor (or move un-exisiting attributes to this anchor) 
    for( auto it = attributes.begin(); it != attributes.end(); ++it) {
      if( this->_attributes.find(it->first) == this->_attributes.end() ) {
	this->_attributes[it->first] = std::move(it->second);
      }
      else {
	this->_attributes[it->first]->update(it->second);
      }
    }
 
    // Save the anchor - the anchor need to be re-acquired within it's life time...
    if( !this->_persistent ) { 
      this->save(db);    // ...in order to be saved to the database
    }
    else {

      // Update the database
      try {

	// Check and update the (unique) symbol 
	if( this->_attributes.find(CATEGORY) != this->_attributes.end() ) {
	  auto it = this->_attributes.find(CATEGORY);
	  std::string symbol = it->second->toString();
	  if( this->_x.compare( 0, symbol.size(), symbol) != 0 ) {
	    bool found = false;
	    for( auto x : this->_history ) {
	      if( x.compare( 0, symbol.size(), symbol) == 0 ) {
		this->_x = x;
		found = true;
		break;
	      }
	    }
	    if( !found ) {
	      this->_history.push_back(this->_x);
	      this->_x = this->generateSymbol( symbol, db);
	      db.update<std::string>( this->_id, "x", this->_x);
	      db.update<std::string>( this->_id, "H", this->_history);
	    }
	  }
	}

	// Update time
	db.update<double>( this->_id, "t", this->_t.toSec());

	// Update all attributes
	std::vector<mongo::Database::Document> docs;
	for( auto it = this->_attributes.begin(); it != this->_attributes.end(); ++it) {
	  mongo::Database::Document subdoc = it->second->serialize();
	  subdoc.add<std::string>( "type", it->second->getTypeStr());
	  docs.push_back(subdoc);	
	}
	db.update<mongo::Database::Document>( this->_id, "attributes", docs);

	// Update all percepts
	docs.clear();
	for( auto it = this->_percepts.begin(); it != this->_percepts.end(); ++it) {
	  mongo::Database::Document subdoc = it->second->serialize();
	  subdoc.add<std::string>( "type", it->second->getTypeStr());
	  docs.push_back(subdoc);	
	}
	db.update<mongo::Database::Document>( this->_id, "percepts", docs);
	
      }
      catch( const std::exception &e ) {
	std::cout << "[Anchor::update]" << e.what() << std::endl;
      }
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

      // Update attributes and percepts
      this->update(db, other->_t, other->_attributes, other->_percepts);
    }
    catch( const std::exception &e ) {
      std::cout << "[Anchor::merge]" << e.what() << std::endl;
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
  void Anchor::fading(int life) {
    while( life != 0  ) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      life--;
      if( this->_persistent ) {
	break;
      }
    }
    
    // Dead anchor - release this object
    if( !this->_persistent ) {
      this->_attributes.clear();
      this->_percepts.clear();
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
