#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <anchor_msgs/DisplayArray.h>
#include <anchor_msgs/AnchorArray.h>

#include <anchoring/anchor_annotation.hpp>

using namespace cv;
using namespace std;
using namespace anchoring;

#define DEBUG 1

// Window name
const char* AnchorAnnotation::window = "Anchor Annotation";
  
// Constructor
AnchorAnnotation::AnchorAnnotation(ros::NodeHandle nh) : _nh(nh), _priv_nh("~"), _lock_screen(false), _processing(false), _idx(-1) {
  
  _object_sub = _nh.subscribe("/objects/classified", 10, &AnchorAnnotation::queue, this);
  _leap_time = ros::Duration(0.0);

  // Create the anchor map
  _anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", "anchordb") );

  cv::namedWindow( window, 1);
  cv::setMouseCallback( window, &clickCb, this);
  cv::createTrackbar( "Negative time warp", window, &this->_time_pos, 100);
}

AnchorAnnotation::~AnchorAnnotation() {
  _objects.clear();
}


//  Init function
void AnchorAnnotation::init(int threads) {
  ROS_WARN("Start loading...");
  this->_anchors->init(threads);
  ROS_WARN("    ...[done]");
}

/* -----------------------------------------
   Receives and queue each incomming 
   scene of object(s).
   --------------------------------------- */
void AnchorAnnotation::queue( const anchor_msgs::ObjectArrayConstPtr &object_ptr ) {

  // Read image from ROS message
  cv_bridge::CvImagePtr cv_ptr;
  try { 
    cv_ptr = cv_bridge::toCvCopy( object_ptr->image,
				  sensor_msgs::image_encodings::BGR8 );
  } catch( const std::exception &e ) {
    ROS_ERROR("[AnchorAnnotation::queue] %s", e.what() );
    return;
  }

  // Get the time
  ros::Time t = object_ptr->header.stamp;
    
  // If a requested queue, save a screen-shot of current scene 
  if( _lock_screen ) {

    if( !_processing ) {

      this->_lock_time = t - this->_leap_time;

      cv_ptr->image.copyTo(this->_img);      

      // Queue the attributes of incoming objects
      for( uint i = 0; i < object_ptr->objects.size(); i++) {

	// Create a map of all object attributes
	AttributeMap attributes;
	try {
	  attributes[DESCRIPTOR] = AttributePtr( new DescriptorAttribute(object_ptr->objects[i].descriptor) );
	  attributes[COLOR] = AttributePtr( new ColorAttribute(object_ptr->objects[i].color) );
	  attributes[SHAPE] = AttributePtr( new ShapeAttribute(object_ptr->objects[i].shape) );
	  attributes[POSITION] = AttributePtr( new PositionAttribute(object_ptr->objects[i].position) );    
	  attributes[CAFFE] = AttributePtr( new CaffeAttribute(object_ptr->objects[i].caffe) ); 
	} catch( const std::exception &e ) {
	  ROS_ERROR("[AnchorAnnotation::queue]%s", e.what() );
	  continue;
	}
	_objects.push_back(std::move(attributes));
	std::vector<std::vector<cv::Point> > contour;
	this->getContour( _objects.back()[CAFFE], contour);
	cv::drawContours( this->_img, contour, -1, cv::Scalar( 0, 0, 255), 1);
	
      }
      _processing = true;
    }
    else {
      this->_leap_time = this->_leap_time + ros::Duration(t.toSec() - this->_time.toSec());
    }
  }
  else {
    cv_ptr->image.copyTo(this->_img);
  }
  this->_time = t;
}

void AnchorAnnotation::sort( map< string, map<anchoring::AttributeType, float> > &matches, int num) {
  std::map< double, string, std::greater<double> > result;
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {
    double dist = (ite->second[CAFFE] + ite->second[COLOR] + ite->second[SHAPE]) / 3.0;
    result[dist] = ite->first;
    #if DEBUG == 1
    std::cout << ite->first << ": [" << ite->second[CAFFE] << ", " << ite->second[COLOR] << ", " << ite->second[SHAPE] << "]" << std::endl;
    #endif
  }
  {
    auto ite = result.begin();
    ite = std::next( ite, num);
    for( ; ite != result.end(); ++ite) {
      matches.erase(ite->second);
    }
  }
  
  #if DEBUG == 1
  std::cout << "Best matches: " << std::endl;
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {
    std::cout << ite->first << std::endl;
  }
  std::cout << "---" << std::endl;
  #endif
}

void AnchorAnnotation::reset(bool complete) {
  if( !complete ) {
    this->_objects.erase( this->_objects.begin() + this->_idx );
  }
  else {
    this->_objects.clear();
  }
  this->_matches.clear();
  this->_idx = -1;
}

// Print instructions
void AnchorAnnotation::help() {
  cout << "Usage:" << endl;
  cout << "----------------------" << endl;
  cout << "  r - Reset the target anchor." << endl;
  cout << "  h - Display this message." << endl;
  cout << "  q - Quit." << endl;
  cout << "----------------------" << endl;
}
  
// Mouse click callback function(s)
void AnchorAnnotation::clickCb(int event, int x, int y, int flags, void *obj) {
  AnchorAnnotation *self = static_cast<AnchorAnnotation*>(obj);
  if( event == cv::EVENT_LBUTTONDOWN ) {
    self->clickWrapper( event, x, y, flags);
  }
}

void AnchorAnnotation::clickWrapper(int event, int x, int y, int flags) {
  if( this->_idx < 0 ) {
    int idx = 0;
    for( auto &object : _objects) {

      // Get and check the contour
      std::vector<std::vector<cv::Point> > contour;
      this->getContour( object[CAFFE], contour);
      if( cv::pointPolygonTest( contour.back(), cv::Point( x, y ), false) > 0 ) {
	cv::drawContours( this->_img, contour, -1, cv::Scalar( 0, 255, 0), 1);
	this->_idx = idx;

	// Saftey check
	if( this->_anchors->empty() ) {
	  break;
	}

	// Match attributes of selected object
	this->_matches.clear();
	this->_anchors->match( object, this->_matches);
	this->sort( this->_matches );

	this->_time_history = 0.0;
	for( auto ite = this->_matches.begin(); ite != this->_matches.end(); ++ite) {
	  double history = this->_anchors->diff( ite->first, this->_lock_time);
	  if( history > this->_time_history ) {
	    this->_time_history = history;
	  }
	}
        #if DEBUG == 1
        std::cout << "Time history: " << this->_time_history << std::endl;
	#endif
	
	cv::setTrackbarPos( "Negative time warp", window, 100);
	break;
      }
      idx++;
    }
  }
  else {
    std::string id = "";
    double dist = 1.0;
    int pose = cv::getTrackbarPos( "Negative time warp", window);  
    for( auto ite = _matches.begin(); ite != _matches.end(); ++ite) {
      cv::Rect roi = this->getRect( this->_anchors->get( ite->first, CAFFE) );
      if( roi.contains( cv::Point( x, y ) ) ) {
	if( this->_time_history > 1.0 ) {
	  double time_diff = this->_anchors->diff( ite->first, this->_lock_time) / this->_time_history;
	  time_diff = std::abs( time_diff - (100 - pose) / 100.0 );
	  if( time_diff < dist ) {
	    dist = time_diff;
	    id = ite->first;
	  }
	}
	else {
	  id = ite->first;	  
	}
      }
    }
    if( !id.empty() ) {
      try {
        #if DEBUG == 1
        std::cout << "Re-aquire: " << id << std::endl;
	#endif
	this->_anchors->re_acquire( id, this->_objects[this->_idx], this->_lock_time ); // RE_ACQUIRE
      }
      catch( const std::exception &e ) {
	ROS_ERROR("[AnchorAnnotation::click]%s", e.what() );
      }
    }
    else {
      try {
	this->_anchors->acquire( this->_objects[this->_idx], this->_lock_time, true); // ACQUIRE
      }
      catch( const std::system_error &e ) {
	ROS_ERROR("[AnchorAnnotation::click]%s", e.what() );
      }	
    }	
    this->reset();
  }
}

cv::Mat AnchorAnnotation::subImages(int idx) {
  
  // Convert to gray image
  cv::Mat result;
  cv::cvtColor( this->_img, result, CV_BGR2GRAY); 
  cv::cvtColor( result, result, CV_GRAY2BGR);
  result.convertTo( result, -1, 1.0, 50); 

  // Iterate and draw sub images
  int pose = cv::getTrackbarPos( "Negative time warp", window);
  for( auto ite = _matches.begin(); ite != _matches.end(); ++ite) {
    cv::Mat roi = result( this->getRect( this->_anchors->get( ite->first, CAFFE) ) );
    cv::Mat img = this->getImage( this->_anchors->get( ite->first, CAFFE) ); 
    if( this->_time_history > 1.0 ) {
      double time_diff = this->_anchors->diff( ite->first, this->_lock_time) / this->_time_history;
      double alpha = std::abs(time_diff - pose / 100.0);
      cv::addWeighted( img, alpha, roi, 1.0 - alpha , 0.0, roi);
    }
    else {
      img.copyTo(roi);
    }
  }

  return result;
}


// ------------------------
// Access functions
// ---------------------------
void AnchorAnnotation::getContour( const anchoring::AttributePtr &ptr, std::vector<std::vector<cv::Point> > &contours) {

  // Typecast the query pointer
  CaffeAttribute *caffe_ptr = dynamic_cast<CaffeAttribute*>(ptr.get());
  assert( caffe_ptr != nullptr );
  
  // Draw the contour
  std::vector<cv::Point> contour;
  for( uint i = 0; i < caffe_ptr->_border.contour.size(); i++) {
    cv::Point p( caffe_ptr->_border.contour[i].x, caffe_ptr->_border.contour[i].y );
    contour.push_back(p);
  }	  
  contours.push_back(contour);
}

cv::Mat AnchorAnnotation::getImage(const anchoring::AttributePtr &ptr) {

  // Typecast the query pointer
  CaffeAttribute *caffe_ptr = dynamic_cast<CaffeAttribute*>(ptr.get());
  assert( caffe_ptr != nullptr );

  return caffe_ptr->_data;
}
cv::Rect AnchorAnnotation::getRect(const anchoring::AttributePtr &ptr) {

  // Typecast the query pointer
  CaffeAttribute *caffe_ptr = dynamic_cast<CaffeAttribute*>(ptr.get());
  assert( caffe_ptr != nullptr );

  return cv::Rect( cv::Point( caffe_ptr->_point.x, caffe_ptr->_point.y),
		   caffe_ptr->_data.size() );		   
}

// For 'ROS loop' access
void AnchorAnnotation::spin() {
  ros::Rate rate(30);
  while(ros::ok()) {

    // OpenCV window for display
    if( !this->_img.empty() ) {
      if( this->_idx < 0 ) {
	cv::imshow( AnchorAnnotation::window, this->_img );
      }
      else {
	cv::imshow( AnchorAnnotation::window, this->subImages(this->_idx) );
      }
    }

    // Wait for a keystroke in the window
    char key = cv::waitKey(1);            
    if( key == 27 || key == 'Q' || key == 'q' ) {
      break;
    }
    else if( key == 32 || key ==  'N' || key == 'n' ) {
      _lock_screen = !_lock_screen;
      this->reset(true);
      _processing = false;
      std::cout << "Screen lock changed." << endl;
    }
    else if( key == 'H' || key == 'h' ) {
      this->help();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

// ------------------------------------------
// Main function
// ------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "anchoring_annotation"); 
  ros::NodeHandle nh;

  // Reading the number of threads used to initilize the anchor space with
  int threads;
  if( !ros::param::get("~threads", threads) ) {
    threads = 8;  // Defualt: 8
  }

  AnchorAnnotation node(nh);
  node.init(threads);

  node.spin();
  return 0;
}
