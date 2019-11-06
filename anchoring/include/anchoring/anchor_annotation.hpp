#ifndef __ANCHOR_ANNOTATINO_HPP__
#define __ANCHOR_ANNOTATINO_HPP__ 

#include <anchor_msgs/ObjectArray.h> 

#include <anchoring/anchor_collection.hpp>

using namespace std;

// ------------------------------------------
// Class for creating and maintaining anchors
// ------------------------------------------
class AnchorAnnotation {
  
  // An object for holding all anchors
  std::unique_ptr<anchoring::AnchorCollection> _anchors;

  // List of attributes of unknown objects
  std::vector<anchoring::AttributeMap> _objects;
  //std::vector< map< string, map<anchoring::AttributeType, float> > > _matches;
  map< string, map<anchoring::AttributeType, float> > _matches;

 
  /* --------------
     ROS variables
     -------------- */  
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;
  ros::Subscriber _object_sub;
  
  ros::Time _time;
  ros::Time _lock_time;
  ros::Duration _leap_time;

  int _time_pos; 
  double _time_history;

  cv::Mat _img;
  int _idx;

  bool _lock_screen;
  bool _processing;
  std::string _db_name;
  int _max_matches;

  void queue( const anchor_msgs::ObjectArrayConstPtr &object_ptr );
  void sort( map< string, map<anchoring::AttributeType, float> > &matches, int num = 5 );
  void save( map< string, map<anchoring::AttributeType, float> > &matches, std::string id = "");
  void reset(bool complete = false);
  
  int process( map< string, map<anchoring::AttributeType, float> > &matches, 
	       string &id, 
	       float dist_th,
	       float rate_th = 0.75 );

  // Display functions
  static const char* window;
  void help();
  static void clickCb(int event, int x, int y, int flags, void *obj);
  void clickWrapper(int event, int x, int y, int flags);
  //void onTrack(int pos, void *obj);
  cv::Mat subImages(int idx);

  // Access functions
  void getContour( const anchoring::AttributePtr &ptr, std::vector<std::vector<cv::Point> > &contours);
  cv::Mat getImage(const anchoring::AttributePtr &ptr);
  cv::Rect getRect(const anchoring::AttributePtr &ptr);

public: 
  AnchorAnnotation(ros::NodeHandle nh);
  ~AnchorAnnotation();

  void init(int threads);
  void spin();
};

#endif // __ANCHOR_MANAGEMENT_HPP__
