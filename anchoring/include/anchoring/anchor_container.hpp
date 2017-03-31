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
    void track(const string &id, const string &corr);
    void acquire(AttributeMap &attributes, const ros::Time &t, bool save = false);
    void re_acquire(const string &id, AttributeMap &attributes, const ros::Time &t, bool track = false);
    void maintain();
    template <typename T> void getArray(vector<T> &array, const ros::Time &t);

    const AttributePtr& get(const string &id, AttributeType type) const {
      auto ite = this->_map.find(id);
      if( ite == this->_map.end() ) {
	throw std::logic_error("[Anchor::getSingle]: there exists no anchor with id '" + id +"'.");
      }
      return ite->second->get(type);
    }
    double diff(const string &id, const ros::Time &t) {
      auto ite = this->_map.find(id);
      if( ite == this->_map.end() ) {
	throw std::logic_error("[Anchor::diff]: there exists no anchor with id '" + id +"'.");
      }
      return t.toSec() - ite->second->getTime();
    }

    // Map access functions
    uint size() { return this->_map.size(); }
    bool empty() { return this->_map.empty(); }
    std::string toString(const string &id) { return this->_map[id]->toString(); } 
  };


  // ----------------------------------------
  // Template functions.
  // ----------------------------------------

  template<typename T> void AnchorContainer::getArray(vector<T> &array, const ros::Time &t) {
    
    // Iterate and get a snapshot of all anchors in current scene
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ++ite) {
      if( ( t.toSec() - ite->second->getTime() ) < 0.01 ) { // ...4.0 sec time diff
	array.push_back(ite->second->getAnchor<T>());
      }
    }
  }
}  

#endif // __ANCHOR_CONTAINER_HPP__
