#ifndef __ANCHOR_COLLECTION_HPP__
#define __ANCHOR_COLLECTION_HPP__

#include <iostream>
#include <iostream>
#include <map>
#include <thread>
#include <mutex>

//#include <database/database.hpp>
#include <anchoring/anchor.hpp>

namespace anchoring {

  using namespace std;

  class AnchorCollection {
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
    AnchorCollection() : _collection(""), _db_name("") {}
    AnchorCollection(const string &collection, const string &db_name);
    ~AnchorCollection();
    bool persistent() {
      return !_collection.empty() && !_db_name.empty();
    }
      
    // Public init och matching functions
    void init(int threads);
    void match( const AttributeMap &attributes,
		map< string, MatchMap> &matches );

    // Public anchoring functions
    void acquire(AttributeMap &attributes, PerceptMap &percepts, const ros::Time &t, bool persistent = false);
    void re_acquire(const string &id, AttributeMap &attributes, PerceptMap &percepts, const ros::Time &t);
    void track(const string &id, AttributeMap &attributes, const ros::Time &t);
    void remove(std::vector<std::string> &removed);

    // Get anchors as different ROS messages
    template <typename T> T get(const string &id);
    template <typename T> void getArray(vector<T> &array, const ros::Time &t = ros::Time::now());

    const AttributePtr& get(const string &id, AttributeType type) const {
      auto ite = this->_map.find(this->resolve(id));
      return ite->second->get(type);
    }
    double diff(const string &id, const ros::Time &t) {
      auto ite = this->_map.find(this->resolve(id));
      return std::abs(t.toSec() - ite->second->time());
    }

    // Map access functions
    bool exist(const string &id) {
      return this->_map.find(id) != this->_map.end();
    }
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
  template <typename T> T AnchorCollection::get(const string &id) {
    auto ite = this->_map.find(id);
    if( ite == this->_map.end() ) {
      throw std::logic_error("[AnchorCollection::get]: there exists no anchor with id '" + id +"'.");
    }
    return ite->second->getAnchor<T>();
  }

  template<typename T> void AnchorCollection::getArray(vector<T> &array, const ros::Time &t) {

    // Get the last time for anchor updates
    double stamp = -1.0;
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ++ite) {
      if( abs(t.toSec() - ite->second->time()) < stamp || stamp < 0.0 ) {
	stamp = abs(t.toSec() - ite->second->time());
      }
    }
    
    // Iterate and get a snapshot of all anchors in current scene 
    for( auto ite = this->_map.begin(); ite != this->_map.end(); ++ite) {
      if( abs(t.toSec() - ite->second->time()) < (stamp + 0.001) ) { // ...only updated anchors.
	array.push_back(ite->second->getAnchor<T>());
      }
    }
  }
}  

#endif // __ANCHOR_COLLECTION_HPP__
