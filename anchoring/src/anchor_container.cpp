/* ------------------------------------------ 

   The class for store and maintain a map
   of anchors.
   ------------------------------------------ */
#include <anchoring/anchor_container.hpp>

namespace anchoring {

  using namespace std;


  // Constructor
  AnchorContainer::AnchorContainer(const string &collection, const string &db_name) : _collection(collection), _db_name(db_name) {
    /*
#ifdef USE_F_SUM
    this->_fsum_1d = new fSum(fSum::SUM_1D);
    this->_fsum_2d = new fSum(fSum::SUM_2D);
#else
    this->_tree = new SearchTree();
#endif
    */
  }


  // Destructor
  AnchorContainer::~AnchorContainer() {
    /*
    // Clean up...
#ifdef USE_F_SUM
    this->_fsum_1d->clear();
    delete this->_fsum_1d;
    this->_fsum_2d->clear();
    delete this->_fsum_2d;
#else
    this->_tree->clear();
    delete this->_tree; 
#endif
    */

    // Clear the object list
    this->_map.clear();
  }


  // ------------------
  // Private helper functions
  // --------------------------------------------------

  // Read all identifiers from db
  void AnchorContainer::identifiers(vector<string> &ids, int size) {
    try {
      mongo::Database::id_array( this->_db_name, this->_collection, ids, size); // Default size = -1 (umlimited)
    }
    catch( const std::exception &e ) {
      std::cout << "[AnchorContainer::identifiers]" << e.what() << std::endl;
    }
  }

  // Split list of identifiers
  void AnchorContainer::split( const vector<string> &list,
			  vector<vector<string> > &split_list,
			  int div ) {
    int chunk = list.size() / div;
    vector<string>::const_iterator first = list.begin(), last;
    for( auto i = 0; i < div; i++) {
      if( i < div - 1 ) {
	last = first + chunk;
      }
      else {
	last = list.end();
      }
      split_list.push_back(vector<string>( first, last ));
      first = first + chunk;
    }
  }

  // Thread init function
  void AnchorContainer::run(const vector<string> &ids) {
    mongo::Database db(this->_db_name, this->_collection);
    try { 
      for( auto ite = ids.begin(); ite != ids.end(); ++ite ) {
	AnchorPtr anchor( new Anchor(*ite) );
	//mongo::Database::Document doc = db.get(*ite); // Load a sub document
	anchor->load(db);  
	this->_mtx.lock();
	this->_map[*ite] = anchor;
	this->_mtx.unlock();
      }
    }
    catch( const std::exception &e ) {
      std::cout << "[AnchorContainer::init]" << e.what() << std::endl;
    }
  }

  /*
  // Function for typcasting and accessing map elements 
  std::shared_ptr<Anchor> AnchorContainer::get(string id) {
    return dynamic_pointer_cast<Anchor>(this->_map[id]);
  }
  */


  // ------------------
  // Public main init function
  // --------------------------------------------------

  void AnchorContainer::init(int threads) {
    
    // Read identifiers
    vector<string> ids;
    this->identifiers(ids);
    ROS_WARN("Init: %d (threads: %d)", (int)ids.size(), threads);
    
    // Start init process(es)
    if( threads < (int)ids.size() ) {
      
      // Split the collection of ideintifers
      vector<vector<string> > splitted;
      this->split( ids, splitted, threads);

      // Start running pool of threads
      for( auto ite = splitted.begin(); ite != splitted.end(); ++ite ) {
	this->_threads.push_back(std::thread( &AnchorContainer::run, this, *ite));
      }
      
      // Wait for threads to finish   
      for( auto ite = this->_threads.begin(); ite != this->_threads.end(); ++ite) {
	if( ite->joinable() ) {
	  ite->join();
	}
      }
      this->_threads.clear();
    }
    else {
      this->run(ids);
    }
    
    // Build the f-sum model
    //this->build();
  }


  // -----------------------
  // Match function
  // -----------------------  
  void AnchorContainer::match( const AttributeMap &attributes,
			       map< string, MatchMap> &matches ) {
    
    // Match agains all attribute (except for the descriptor)
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ++ite ) {
      MatchMap match;
      ite->second->match(attributes, match);
      matches[ite->first] = match;
    }
  }

  
  // ------------------
  // Anchoring functions - maintain long term memeory
  // --------------------------------------------------
  
  // Track (by position) an exisitng anchor 
  void AnchorContainer::track(const string &id, AttributeMap &attributes, const ros::Time &t) {
    mongo::Database db(this->_db_name, this->_collection);
    this->_map[id]->maintain( db, attributes, t);
    ROS_WARN("[Anchor (tracked): %s", this->_map[id]->toString().c_str());
  }

  // Track (or correct) an existing anchor based on data association 
  void AnchorContainer::track(const string &id, const string &corr) {
    mongo::Database db(this->_db_name, this->_collection);
    this->_map[id]->merge( db, this->_map[corr]);
    ROS_WARN("[Anchor (tracked): %s", this->_map[id]->toString().c_str());

    // Delete the 'glitch' anchor
    db.remove(corr);
    this->_map.erase(corr);  
  }

  // Acquire a new anchor
  void AnchorContainer::acquire(AttributeMap &attributes, const ros::Time &t, bool save) {
    AnchorPtr anchor( new Anchor(attributes, t) ); // Generates an unique id for the anchor as well...
    if( save ) {
      mongo::Database db(this->_db_name, this->_collection);
      anchor->create(db);
    }
    string id = anchor->getId();
    this->_map[id] = anchor;
    //this->add(id); // Add to the binary descriptor model
    ROS_WARN("[Anchor (acquired): %s", this->_map[id]->toString().c_str());
  }
  
  // Re-acquire an exisitng anchor
  void AnchorContainer::re_acquire(const string &id, AttributeMap &attributes, const ros::Time &t, bool track ) {
    mongo::Database db(this->_db_name, this->_collection);
    this->_map[id]->maintain( db, attributes, t);
    ROS_WARN("[Anchor (re-acquired): %s", this->_map[id]->toString().c_str());
  }
  
  // Maintain the anchor space
  void AnchorContainer::maintain() {
    
    // Remove glitches
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ) {
      if( ite->second->invalid() ) {
	ROS_WARN("Anchor [removed]: %s", ite->first.c_str());
	//this->remove(ite->first); // Remove from the binary descriptor model
	ite = this->_map.erase( ite );
      }
      else {
	++ite;
      }      
    }
  }

} // namespace anchoring
