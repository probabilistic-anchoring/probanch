#ifndef __ANCHOR_CONTAINER_HPP__
#define __ANCHOR_CONTAINER_HPP__

#include <iostream>
#include <iostream>
#include <map>
#include <thread>
#include <mutex>

#include <anchoring/anchor.hpp>

namespace anchoring {

  using namespace std;

  class AnchorContainer {
  private:

    // Database name and collection
    string _collection;
    string _db_name;

    // The main map of anchors
    map< string, AnchorPtr> _map;

    // Functions for reading anchor identifieres 
    void identifiers(vector<string> &ids, int size = 0);
    void split( const vector<string> &list,
		vector<vector<string> > &split_list,
		int div );

    // Thread variables and function
    vector<std::thread> _threads; // Pool of threads 
    std::mutex _mtx;
    virtual void run(const vector<string> &ids);
    
    /*
    // Binary visual feature matching objects
#ifdef USE_F_SUM
    fSum  *_fsum_1d, *_fsum_2d;
#else
    SearchTree *_tree;
    int _idx;
    map<string, int> _id_mapper;
#endif

    // Binary visual feature functions
    void build();
    void add(const string &id);
    void remove(const string &id);
    */
   
  public:
    AnchorContainer(const string &collection, const string &db_name);
    ~AnchorContainer();

    // Public init och matching functions
    void init(int threads);
    void match( const AttributeMap &attributes,
		map< string, MatchMap> &matches );

    // Public anchoring functions
    void track(const string &id, AttributeMap &attributes, const ros::Time &t);
    void track(const string &id, const string &other);
    void acquire(AttributeMap &attributes, const ros::Time &t, bool create_persistent = false);
    void re_acquire(const string &id, AttributeMap &attributes, const ros::Time &t, bool track = false);
    void maintain();

    // Get anchors as different ROS messages
    template <typename T> T get(const string &id);
    template <typename T> void getArray(vector<T> &array, const ros::Time &t);

    const AttributePtr& get(const string &id, AttributeType type) const {
      auto ite = this->_map.find(this->resolve(id));
      return ite->second->get(type);
    }
    double diff(const string &id, const ros::Time &t) {
      auto ite = this->_map.find(this->resolve(id));
      return abs(t.toSec() - ite->second->time());
    }

    // Map access functions
    uint size() { return this->_map.size(); }
    bool empty() { return this->_map.empty(); }
    std::string resolve(const string &id) const;
    std::string toString(const string &id) {
      return this->_map[this->resolve(id)]->toString();
    }

  };


  // ----------------------------------------
  // Template functions.
  // ----------------------------------------
  template <typename T> T AnchorContainer::get(const string &id) {
    auto ite = this->_map.find(id);
    if( ite == this->_map.end() ) {
      throw std::logic_error("[Anchor::get]: there exists no anchor with id '" + id +"'.");
    }
    return ite->second->getAnchor<T>();
  }

  template<typename T> void AnchorContainer::getArray(vector<T> &array, const ros::Time &t) {
    
    // Iterate and get a snapshot of all anchors in current scene
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ++ite) {
      if( abs(t.toSec() - ite->second->time()) < 0.001 ) { // ...only updated anchors.
      //if( abs(t.toSec() - ite->second->time()) < 5.0 ) { // ...only updated anchors.
	array.push_back(ite->second->getAnchor<T>());
      }
    }
  }
}  

#endif // __ANCHOR_CONTAINER_HPP__
