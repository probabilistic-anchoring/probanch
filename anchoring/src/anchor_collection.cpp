/* ------------------------------------------ 

   The class for store and maintain a map
   of anchors.
   ------------------------------------------ */
#include <anchoring/anchor_collection.hpp>

namespace anchoring {

  using namespace std;


  // Constructor
  AnchorCollection::AnchorCollection(const string &collection, const string &db_name) : _collection(collection), _db_name(db_name) {
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
  AnchorCollection::~AnchorCollection() {
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
  void AnchorCollection::identifiers(vector<string> &ids, int size) {
    try {
      mongo::Database::id_array( this->_db_name, this->_collection, ids, size); // Default size = -1 (umlimited)
    }
    catch( const std::exception &e ) {
      std::cout << "[AnchorCollection::identifiers]" << e.what() << std::endl;
    }
  }

  // Split list of identifiers
  void AnchorCollection::split( const vector<string> &list,
			  vector<vector<string> > &splitted,
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
      splitted.push_back(vector<string>( first, last ));
      first = first + chunk;
    }
  }

  
  // ------------------
  // Public main init function
  // --------------------------------------------------

  void AnchorCollection::init(int threads) {
    
    // Read identifiers
    vector<string> ids;
    this->identifiers(ids);
    ROS_WARN("Init: %d (threads: %d)", (int)ids.size(), threads);
    
    // Start init process(es)
    if( threads > 0 && threads < (int)ids.size() ) {
      
      // Split the collection of ideintifers
      vector<vector<string> > splitted;
      this->split( ids, splitted, threads);

      // Thread variables
      std::mutex mtx;
      std::vector<std::thread> threads;

      // Start running pool of threads
      for( auto ite = splitted.begin(); ite != splitted.end(); ++ite ) {
	auto run = [&](const vector<string> &ids) {
	  mongo::Database db(this->_db_name, this->_collection);
	  try { 
	    for( auto &id : ids ) {
	      AnchorPtr anchor( new Anchor(id) );
	      anchor->load(db);  
	      mtx.lock();
	      this->_map[id] = anchor;
	      mtx.unlock();
	    }
	  }
	  catch( const std::exception &e ) {
	    std::cout << "[AnchorCollection::init]" << e.what() << std::endl;
	  } 
	};
	std::thread runner{run, *ite};
	threads.push_back(std::move(runner));
      }

      // Wait for threads to finish
      for (auto&& runner : threads) {
        runner.join();
      }
    }
    else {
      mongo::Database db(this->_db_name, this->_collection);
      try { 
	for( auto &id : ids ) {
	  AnchorPtr anchor( new Anchor(id) );
	  anchor->load(db);  
	  this->_map[id] = anchor;
	}
      }
      catch( const std::exception &e ) {
	std::cout << "[AnchorCollection::init]" << e.what() << std::endl;
      } 
    }
    
    // Build the f-sum model
    //this->build();
  }


  // -----------------------
  // Match function
  // -----------------------  
  void AnchorCollection::match( const AttributeMap &attributes,
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

  // Acquire a new anchor
  void AnchorCollection::acquire(AttributeMap &attributes, PerceptMap &percepts, const ros::Time &t, bool persistent) {
    AnchorPtr anchor( new Anchor(t, attributes, percepts) ); // Generates an unique id for the anchor as well...
    if( persistent ) {
      if( !_collection.empty() && !_db_name.empty() ) { 
	mongo::Database db(this->_db_name, this->_collection);
	anchor->save(db);
      }
      else {
	mongo::Database db;
	anchor->save(db);
      }
    }
    string id = anchor->id();
    this->_map[id] = anchor;
    //this->add(id); // Add to the binary descriptor model
    //ROS_WARN("[Anchor (acquired): %s (%s)]", id.c_str(), this->_map[id]->toString().c_str());

  }
  
  // Re-acquire an exisitng anchor
  void AnchorCollection::re_acquire(const string &id, AttributeMap &attributes, PerceptMap &percepts, const ros::Time &t ) {
    if( !_collection.empty() && !_db_name.empty() ) {
      mongo::Database db(this->_db_name, this->_collection);
      this->_map[id]->update( db, t, attributes, percepts);
    }
    else {
      mongo::Database db;
      this->_map[id]->update( db, t, attributes, percepts);
    }
    //ROS_WARN("[Anchor (re-acquired): %s (%s)]", id.c_str(), this->_map[id]->toString().c_str());
  }
    
  // Track (by position) an exisitng anchor 
  void AnchorCollection::track(const string &id, AttributeMap &attributes, const ros::Time &t) {
    if( !_collection.empty() && !_db_name.empty() ) {
      mongo::Database db(this->_db_name, this->_collection);
      this->_map[id]->update( db, t, attributes);
    }
    else {
      mongo::Database db;
      this->_map[id]->update( db, t, attributes);
    }
    //ROS_WARN("[Anchor (tracked): %s (%s)]", id.c_str(), this->_map[id]->toString().c_str());
  }

  // Remove the anchor space
  void AnchorCollection::remove(std::vector<std::string> &removed) {
    
    // Remove glitches
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ) {
      if( ite->second->expired() ) {
	ROS_WARN("Anchor [removed]: %s", ite->first.c_str());
	//this->remove(ite->first); // Remove from the binary descriptor model
	removed.push_back(ite->first);
	ite = this->_map.erase( ite );
      }
      else {
	++ite;
      }      
    }
  }

  // Find an anchor (the safe way)
  std::string AnchorCollection::resolve(const string &id) const {
    auto ite = this->_map.find(id);
    if( ite == this->_map.end() ) {
      ite = this->_map.begin();
      for( ; ite != this->_map.end(); ++ite ) {
	if( ite->second->merged(id) ) {
	  return ite->second->id();
	}
      }
      throw std::logic_error("[AnchorCollection::find] There exists no anchor with id '" + id +"'.");
    }
    return id;
  }

} // namespace anchoring
