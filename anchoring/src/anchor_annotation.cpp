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
  
// Constructor
AnchorAnnotation::AnchorAnnotation(ros::NodeHandle nh) : _nh(nh), _priv_nh("~"), _lock_screen(false), _processing(false) {
  
  _object_sub = _nh.subscribe("/objects/classified", 10, &AnchorAnnotation::queue, this);

  // Create the anchor map
  _anchors = std::unique_ptr<AnchorContainer>( new AnchorContainer("anchors", "anchordb") );

  //  *AnchorAnnotation::window = ;
  cv::namedWindow( "Anchor annotation", 1);
  cv::setMouseCallback( "Anchor annotation", &click_cb, this);
}

AnchorAnnotation::~AnchorAnnotation() {
  /*
  for( auto &&ite : _objects) {
    ite.clear();
  }
  _objects.clear();
  */
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
    
  // If a requested queue, save a screen-shot of current scene 
  if( _lock_screen ) {

    // Get the time
    ros::Time t = object_ptr->header.stamp;
    this->_t = t.toSec();

    if( !_processing ) {

      cv::cvtColor( cv_ptr->image, this->_img, CV_BGR2GRAY); 
      cv::cvtColor( this->_img, this->_img, CV_GRAY2BGR);
      this->_img.convertTo( this->_img, -1, 1.0, 50); 


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
	//_objects.push_back(attributes);
      }

      _processing = true;
    }
  }
  else {
    cv_ptr->image.copyTo(this->_img);
  }
  

  /*
  // Match all attributes
  map< string, map<anchoring::AttributeType, float> > matches;
  this->_anchors->match( attributes, matches);

  // Process matches
  string id;
  int result = this->process( matches, id, 0.95, 0.65);
  if( result != 0 ) {
  this->_anchors->re_acquire(id, attributes, t, (result > 0 ? true : false) ); // RE_ACQUIRE
  }
  else {
  this->_anchors->acquire(attributes, t); // ACQUIRE
  } 
  */
  

  /*
  // Get a snapshot of all anchors seen scene at time t
  anchor_msgs::AnchorArray msg;
  this->_anchors->getArray<anchor_msgs::Anchor>( msg.anchors, t );
  //this->_anchor_pub.publish(msg);
  */
}

/* -----------------------------------------
   Process function  
   -----------------
   Process all matches and return true in 
   case an object is re-observed.
   ---
   All matching values are in range 
   [0.0 1.0] (where 1.0 is a perfect match).
   ---

   Return:
     -1 - tracked by distance
      0 - no match
     +1 - re-acquried by match
   -------------------------------------- */
int AnchorAnnotation::process( map< string, map<AttributeType, float> > &matches, 
			       string &id, 
			       float dist_th,
			       float rate_th ) {
 
  // Check the location (main feature for track)
  float best = 0.0;
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {
    //std::cout << "Prob. dist: " << ite->second[POSITION] << ", prob. rate: " << ite->second[IMAGE] << std::endl;
    if( ite->second[POSITION] > best ) {
      best = ite->second[POSITION];
      id = ite->first;
    }
  }
  if( best > dist_th ) {
    return -1;
  }

  // Check keypoints, caffe-result, color and shape (main feature for acquire/re_acquire)
  best = 0.0;
  for( auto ite = matches.begin(); ite != matches.end(); ++ite) {

    // ( DESCRIPTOR OR CAFFE ) AND ( COLOR AND SHAPE )
    float rate_1 = max( ite->second[DESCRIPTOR], ite->second[CAFFE] ); 
    float rate_2 = min( ite->second[COLOR], ite->second[SHAPE] );      
    float rate = min( rate_1, rate_2 ); 
    if( rate > best ) {
      best = rate;      
      id = ite->first;
    }
  }
  if( best > rate_th ) {
    return 1;
  }

  return 0;
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
void AnchorAnnotation::click_cb(int event, int x, int y, int flags, void *obj) {
  AnchorAnnotation *self = static_cast<AnchorAnnotation*>(obj);
  if( event == cv::EVENT_LBUTTONDOWN ) {
    self->click_wrapper( event, x, y, flags);
  }
}

void AnchorAnnotation::click_wrapper(int event, int x, int y, int flags) {
  cout << "CLicked - x: " << x << ", y: " << y << endl; 
  /*
  for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {
    
    // Get the contour
    std::vector<cv::Point> contour;
    for( uint i = 0; i < ite->border.contour.size(); i++) {
      cv::Point p( ite->border.contour[i].x, ite->border.contour[i].y );
      contour.push_back(p);
    }	  
    if( cv::pointPolygonTest( contour, cv::Point( x, y ), false) > 0 ) {
      this->_highlight = ite->id;
      ROS_WARN("Anchored selected at: x = %.2f, y = %.2f, z = %.2f", (float)ite->pos.pose.position.x, (float)ite->pos.pose.position.y, (float)ite->pos.pose.position.z);
      break;
      }
  }
  */
}

// For 'ROS loop' access
void AnchorAnnotation::spin() {
  ros::Rate rate(30);
  while(ros::ok()) {

    // OpenCV window for display
    if( !this->_img.empty() ) {
      cv::imshow( "Anchor annotation", this->_img );
      
      //cv::imshow( window, this->anchor_img() );
    }

    // Wait for a keystroke in the window
    char key = cv::waitKey(1);            
    if( key == 27 || key == 'Q' || key == 'q' ) {
      break;
    }
    else if( key == 32 || key ==  'N' || key == 'n' ) {
      _lock_screen = !_lock_screen;
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
