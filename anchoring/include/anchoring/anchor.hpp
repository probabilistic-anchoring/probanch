#ifndef __ANCHOR_HPP__
#define __ANCHOR_HPP__

#include <map>
#include <memory>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>

#include <anchoring/attribute.hpp>

namespace anchoring {

  using namespace std;

  class Anchor {
  public:
    
    // Public functions
    Anchor(const string &id) : _id(id), _aging(false) {} 
    Anchor(AttributeMap &attributes, const ros::Time &t);
    ~Anchor();

    // Anchoring helper functions
    void load(const mongo::Database &db); 
    void create(mongo::Database &db); 
    void maintain(mongo::Database &db, AttributeMap &attributes, const ros::Time &t);
    void merge(mongo::Database &db, std::shared_ptr<Anchor> &other);
    bool invalid() { return this->_attributes.empty(); }
    
    // Matching function
    void match( const AttributeMap &attributes,
		MatchMap &result ); 

    // Get/set functions 
    void setSymbols( AttributeType type, const vector<string> &symbols); 
    vector<string> getSymbols(AttributeType type);
    string getId() { return this->_id; }
    double getTime() { return this->_t.toSec(); }
    string getTimeStr();
    
    string toString();
    template <typename T> T getAnchor();
    
    const AttributePtr& get(AttributeType type) const {
      return this->_attributes.at(type);
    }
    
  private:

    // ---[ Private update functions ]---
    template <typename Map> bool compare(Map const &lhs, Map const &rhs);
    void append(mongo::Database &db, AttributeMap &attributes);
    void update(mongo::Database &db, AttributeMap &attributes);

    // Unique id for each anchor
    string _id;

    // Unique symbol for each anchor
    string _x;
    
    // Features 
    AttributeMap _attributes;

    // Time
    ros::Time _t;
       
    // Variables and functions for maintenance
    std::thread _thread;
    std::mutex _mtx;
    bool _aging;
    void decreaseAnchor(int life);

    string generate_symbol(const string &key, mongo::Database &db);

  };

  // Typedefine a smart object pointer
  typedef std::shared_ptr<Anchor> AnchorPtr; 

  // ----------------------------------------
  // Template functions.
  // ----------------------------------------
  template <typename T> T Anchor::getAnchor() {
    T msg;
    msg.id = this->_id;
    msg.x = this->_x;
    msg.t = this->_t;
    for( auto ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
      ite->second->populate(msg);
    }
    return msg;
  }

  // Compare and check if all keys of two maps are the same
  template <typename Map> bool Anchor::compare(Map const &lhs, Map const &rhs) {

    auto pred = [] (decltype(*lhs.begin()) a, decltype(a) b)
      { return a.first == b.first; };
    
    return lhs.size() == rhs.size()
      && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
  }
}

#endif // __ANCHOR_HPP__
