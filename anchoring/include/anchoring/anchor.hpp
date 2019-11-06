#ifndef __ANCHOR_HPP__
#define __ANCHOR_HPP__

#include <map>
//#include <memory>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>

#include <anchoring/attribute.hpp>
#include <anchoring/percept.hpp>

namespace anchoring {

  using namespace std;

  class Anchor {
  public:
    
    // Public functions
    Anchor(const string &id) : _id(id), _persistent(true) {} 
    Anchor(const ros::Time &t, AttributeMap &attributes, PerceptMap &percepts);
    ~Anchor();

    // Anchoring helper functions
    void load(const mongo::Database &db); 
    void save(mongo::Database &db); 
    void update(mongo::Database &db, const ros::Time &t, AttributeMap &attributes, PerceptMap &percepts);
    void update(mongo::Database &db, const ros::Time &t, AttributeMap &attributes);
    void merge(mongo::Database &db, std::shared_ptr<Anchor> &other);
    
    bool expired() { return this->_attributes.empty() || this->_percepts.empty(); }
    bool merged(const string &id) { return std::find( _history.begin(), _history.end(), id) != _history.end(); }

    // Matching function
    void match( const AttributeMap &attributes,
		MatchMap &result ); 

    // Get/set functions 
    void setSymbols( AttributeType type, const vector<string> &symbols); 
    vector<string> getSymbols(AttributeType type);

    string id() { return this->_id; }
    double time() { return this->_t.toSec(); }
    string timeStr();
    
    string toString();
    template <typename T> T getAnchor();
    
    const AttributePtr& get(AttributeType type) const {
      return this->_attributes.at(type);
    }
    const AttributeMap& getAll() const {
      return this->_attributes;
    }
    
  private:

    // ---[ Private update functions ]---
    //template <typename Map> bool compare(Map const &lhs, Map const &rhs);

    // Unique id for each anchor
    string _id;
    vector<string> _history;

    // Unique symbol for each anchor
    string _x;
    
    // Attributes and percepts 
    AttributeMap _attributes;
    PerceptMap _percepts;

    // Time
    ros::Time _t;
       
    // Variables and functions for maintenance
    std::thread _thread;
    std::mutex _mtx;
    bool _persistent;
    void fading(int life);

    string generateSymbol(const string &key, mongo::Database &db);

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
    for( auto ite = this->_percepts.begin(); ite != this->_percepts.end(); ++ite) {
      ite->second->populate(msg);
    }
    return msg;
  }

  /*
  // Compare and check if all keys of two maps are the same
  template <typename Map> bool Anchor::compare(Map const &lhs, Map const &rhs) {

    auto pred = [] (decltype(*lhs.begin()) a, decltype(a) b)
      { return a.first == b.first; };
    
    return lhs.size() == rhs.size()
      && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
  }
  */
}

#endif // __ANCHOR_HPP__
