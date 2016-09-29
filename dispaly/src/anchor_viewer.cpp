#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <anchor_msgs/DisplayArray.h>

using namespace std;

class AnchorViewer {

  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;
  ros::Subscriber _anchor_sub;

  cv::Mat _img;
  vector<anchor_msgs::Display> _anchors;

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

  cv::Mat anchor_img() {
    return this->_img;
    
  }

  void help() {
    cout << "Usage:" << endl;
    cout << "----------------------" << endl;
    cout << "  r - Reset the target anchor." << endl;
    cout << "  h - Display this message." << endl;
    cout << "  q - Quit." << endl;
    cout << "----------------------" << endl;
  }

public:
  AnchorViewer(ros::NodeHandle nh) : _nh(nh), _priv_nh("~") {
    
    // ROS subscriber
    _anchor_sub = _nh.subscribe("/dispaly/anchors", 10, &AnchorViewer::display_cb, this);

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
	// Reset the target anchor	
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
