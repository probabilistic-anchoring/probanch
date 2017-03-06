#ifndef __ANCHOR_ANNOTATINO_HPP__
#define __ANCHOR_ANNOTATINO_HPP__ 

#include <anchor_msgs/ObjectArray.h> 

#include <anchoring/anchor_container.hpp>

using namespace std;

// ------------------------------------------
// Class for creating and maintaining anchors
// ------------------------------------------
class AnchorAnnotation {
  
  // An object for holding all anchors
  std::unique_ptr<anchoring::AnchorContainer> _anchors;

  // List of attributes of unknown objects
  std::vector<anchoring::AttributeMap> _objects;
  std::vector< map< string, map<anchoring::AttributeType, float> > > _matches;

 
  /* --------------
     ROS variables
     -------------- */  
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;
  ros::Subscriber _object_sub;
  
  double _t;
  double _leap_time;
  cv::Mat _img;
  bool _lock_screen;
  bool _processing;

  void queue( const anchor_msgs::ObjectArrayConstPtr &object_ptr );
  void sort( map< string, map<anchoring::AttributeType, float> > &matches, int num = 5 );

  
  int process( map< string, map<anchoring::AttributeType, float> > &matches, 
	       string &id, 
	       float dist_th,
	       float rate_th = 0.75 );

  // Display functions
  static const char* window;
  void help();
  static void click_cb(int event, int x, int y, int flags, void *obj);
  void click_wrapper(int event, int x, int y, int flags);

  // ---------------------------------------
  // Template functions
  // --------------------------------------------
  template<typename A, typename B>
  std::pair<B,A> flip_pair(const std::pair<A,B> &p) {
    return std::pair<B,A>(p.second, p.first);
  }

  template<typename A, typename B>
  std::multimap<B,A> flip_map(const std::map<A,B> &src){
    std::multimap<B,A> dst;
    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()), 
                   flip_pair<A,B>);
    return dst;
  }

public: 
  AnchorAnnotation(ros::NodeHandle nh);
  ~AnchorAnnotation();

  void init(int threads);
  void spin();
};

#endif // __ANCHOR_MANAGEMENT_HPP__
