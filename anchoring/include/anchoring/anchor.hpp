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
    Anchor(const string &id) : _id(id) {} 
    Anchor(AttributeMap &attributes, const ros::Time &t);
    ~Anchor();

    // Anchoring helper functions
    void load(const mongo::Database::Document &doc); 
    void update(mongo::Database &db, AttributeMap &attributes, const ros::Time &t, bool append = false);
    void append(AttributeMap &attributes, const ros::Time &t);
    bool invalid() { return this->_attributes.empty(); }
    
    // Matching function
    void match( const AttributeMap &attributes,
		MatchMap &result ); 

    // Get/set functions 
    //cv::Mat get(AttributeType type);
    void setSymbols( AttributeType type, const vector<string> &symbols); 
    vector<string> getSymbols(AttributeType type);
    string getId() { return this->_id; }
    double getTime() { return this->_t.toSec(); }
    string getTimeStr();
    
    string toString();
    template <typename T> T getAnchor();
    
  private:
    
    // Private functions
    void save(mongo::Database &db); 

    // Unique id for each anchor
    string _id;
    
    // Features 
    AttributeMap _attributes;

    // Time
    ros::Time _t;
       
    // Variables and functions for maintenance
    std::thread _thread;
    std::mutex _mtx;
    bool _aging;
    void decreaseAnchor(int life);
  };

  // Typedefine a smart object pointer
  typedef std::shared_ptr<Anchor> AnchorPtr; 

  // ----------------------------------------
  // Template functions.
  // ----------------------------------------
  template <typename T>  T Anchor::getAnchor() {
    T msg;
    msg.id = this->_id;
    msg.t = this->_t;
    for( auto ite = this->_attributes.begin(); ite != this->_attributes.end(); ++ite) {
      ite->second->populate(msg);
    }
    return msg;
  }
}

#endif // __ANCHOR_HPP__
