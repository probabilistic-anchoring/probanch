#include <iostream>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <anchor_msgs/DisplayArray.h>

using namespace std;

class AnchorViewer {

  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;
  ros::Subscriber _anchor_sub;
  ros::Subscriber _highlight_sub;

  cv::Mat _img;
  vector<anchor_msgs::Display> _anchors;
  string _highlight;

  // Callback function for reciving and displaying the image
  void display_cb(const anchor_msgs::DisplayArrayConstPtr& msg_ptr) {

    // Store the image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( msg_ptr->image, 
				    sensor_msgs::image_encodings::BGR8 ); 
      cv_ptr->image.copyTo(this->_img);  
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("[AnchorViewer::display_cb] Could not convert image.");
    }

    // Store the anchor information
    this->_anchors = msg_ptr->anchors;
  }

  // Function for processing the scene image 
  cv::Mat anchor_img() {

    // Draw the result
    cv::Mat result_img(this->_img);
    cv::Mat highlight_img(this->_img);
    for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {

      // Draw the contour
      std::vector<cv::Point> contour;
      for( uint i = 0; i < ite->border.contour.size(); i++) {
	cv::Point p( ite->border.contour[i].x, ite->border.contour[i].y );
	contour.push_back(p);
      }	  
      std::vector<std::vector<cv::Point> > contours;
      contours.push_back(contour);

      // Draw highlighted result
      if( ite->id == this->_highlight ) {
	cv::drawContours( highlight_img, contours, -1, cv::Scalar( 0, 255, 0), CV_FILLED);
      }
      cv::drawContours( result_img, contours, -1, cv::Scalar( 0, 0, 255), 2);
      cv::drawContours( highlight_img, contours, -1, cv::Scalar( 0, 0, 255), 2);

      // Get the bounding box
      cv::Rect rect = cv::boundingRect(contour); 
      std::stringstream ss;
      ss << "Object: " << ite->object << " (" << ite->prediction * 100.0 << "%)";
      cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 58), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,255), 2, 8);
      cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 58), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,255), 2, 8);
      ss.str("");
      ss << "Color(s): [";
      for( uint i = 0; i < ite->colors.size(); i++) {
	ss << ite->colors[i];
	if( i < ite->colors.size() - 1 )
	  ss <<",";
      }
      ss << "]";
      cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 34), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,255), 2, 8);
      cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 34), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,255), 2, 8);
      ss.str("");
      ss << "Size: " << ite->size;
      cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 10), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,255), 2, 8);
      cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 10), cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0,0,255), 2, 8);
      ss.str("");

    } 
    cv::addWeighted( result_img, 0.5, highlight_img, 0.5, 0.0, result_img);
    return result_img;
  }

  // Callback function for reciving and displaying the image
  void highlight_cb(const std_msgs::StringConstPtr& msg) {
    this->_highlight = msg->data;
  }

  void help() {
    cout << "Usage:" << endl;
    cout << "----------------------" << endl;
    cout << "  r - Reset the target anchor." << endl;
    cout << "  h - Display this message." << endl;
    cout << "  q - Quit." << endl;
    cout << "----------------------" << endl;
  }
  
  // Mouse click callback function(s)
  static void click_cb(int event, int x, int y, int flags, void *obj) {
    AnchorViewer *self = static_cast<AnchorViewer*>(obj);
    self->click_wrapper( event, x, y, flags);
  }
  void click_wrapper(int event, int x, int y, int flags) {
    for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {

      // Get the contour
      std::vector<cv::Point> contour;
      for( uint i = 0; i < ite->border.contour.size(); i++) {
	cv::Point p( ite->border.contour[i].x, ite->border.contour[i].y );
	contour.push_back(p);
      }	  
      if( cv::pointPolygonTest( contour, cv::Point( x, y ), false) > 0 ) {
	this->_highlight = ite->id;
      }
    }
  }

public:
  AnchorViewer(ros::NodeHandle nh) : _nh(nh), _priv_nh("~") {
    
    // ROS subscriber
    _anchor_sub = _nh.subscribe("/dispaly/anchors", 10, &AnchorViewer::display_cb, this);
    _highlight_sub = _nh.subscribe("/dispaly/hightlight", 10, &AnchorViewer::highlight_cb, this);
    
    const char *window = "Anchors with information...";
    cv::namedWindow( window, 1);
    cv::setMouseCallback( window, &AnchorViewer::click_cb, this);
  }
  ~AnchorViewer() {}

  void spin() {
    ros::Rate rate(15);
    while(ros::ok()) {

      // OpenCV window for display
      if( !this->_img.empty() ) {
	cv::imshow( "Anchors with information...", this->anchor_img() );
      }

      // Wait for a keystroke in the window
      char key = cv::waitKey(1);            
      if( key == 27 || key == 'Q' || key == 'q' ) {
	break;
      }
      else if( key == 'H' || key == 'h' ) {
	this->help();
      }
      else if( key == 'R' || key == 'r' ) {
	this->_highlight = "";
      }

      ros::spinOnce();
      rate.sleep();
    }
  }
};

// ------------------------------------------
// Main function
// ------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "anchor_viewer_node");
  ros::NodeHandle nh;
  AnchorViewer node(nh);
  node.spin();
  return 0;
}
