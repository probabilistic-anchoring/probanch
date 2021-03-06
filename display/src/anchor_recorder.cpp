#include <iostream>
#include <vector>
#include <sstream>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>

using namespace std;

class AnchorRecorder {

  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;

  image_transport::ImageTransport _it;
  image_transport::Subscriber _img_sub;
  ros::Publisher _trigger_pub;

  cv::Mat _img;
  std::string _trigger_str;
  bool _recording;

  // Callback function for reciving an image frame
  void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( msg_ptr, sensor_msgs::image_encodings::BGR8);
      //cv_ptr->image.copyTo(this->_img);
      this->_img = textImg(cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void triggerPub(const std::string &str) {
    this->_trigger_str = str;
    
    // Publish a trigger message
    std_msgs::String msg;
    msg.data = str;
    _trigger_pub.publish(msg);
  }

  std::string getCation() {
    std::string result = "";
    if( this->_trigger_str == "segmentation") {
      result = "Object Segmentation";
    }
    else if( this->_trigger_str == "tracking" ) {
      result = "Object Tracking";
    }
    else if( this->_trigger_str == "classification" ) {
      result = "Object Classification";
    }
    else if( this->_trigger_str == "grounding" ) {
      result = "Feature Extraction & Symbol Grounding";
    }
    else if( this->_trigger_str == "anchoring" ) {
      result = "Object Anchoring";
    }
    return result;
  }

  std::string getDescription() {
    std::string result = "";
    if( this->_trigger_str == "segmentation") {
      result = "  ...detect and segment objects of interest from a stream of RGB-D data given by a Kinect v2 sensor.";
    }
    else if( this->_trigger_str == "tracking" ) {
      result = "  ..tracks segmented objects on a perceptual level.";
    }
    else if( this->_trigger_str == "classification" ) {
      result = "  ...classifies objects, through the use of a Convolutional Neural Network (CNN), with the goal of symbolically associating a category label with each object.";
    }
    else if( this->_trigger_str == "grounding" ) {
      result = "  ...extracts measurable attributes, e.g. color histograms, and establishes the connection between measured attributes and semantic symbols, e.g. a certain peek in a color histogram is associated with the symbol 'red'.";
    }
    else if( this->_trigger_str == "anchoring" ) {
      result = "  ...creates and maintains consistent representations of objects, both in time and space.";
    }
    return result;
  }

  cv::Mat textImg(const cv::Mat &img) {

    // Draw the result image
    cv::Mat result(img);

    // Print infromation
    cv::putText( result, this->getCation(), cv::Point( 20, 42), cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar( 57, 104, 205), 2, CV_AA);
    //cv::putText( result, this->getDescription(), cv::Point( 20, 72), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar::all(32), 1, CV_AA);

    return result;
  }
  
  // Helper functions
  void help() {
    cout << "Usage:" << endl;
    cout << "----------------------" << endl;
    cout << "  h - Display this message." << endl;
    cout << "  s - Start/stop recording." << endl;
    cout << "  q - Quit." << endl;
    cout << "----------------------" << endl;
  }
  
public:
  AnchorRecorder(ros::NodeHandle nh) : _nh(nh), _priv_nh("~"), _it(nh), _recording(false) {
    
    // ROS subscriber/publisher
    _img_sub = _it.subscribe("/display/image", 1, &AnchorRecorder::imageCb, this);
    _trigger_pub = _nh.advertise<std_msgs::String>("/display/trigger", 1);
    
    // Publish an intial trigger message
    this->triggerPub("anchoring");

    const char *window = "Anchors demo...";
    cv::namedWindow( window, 1);
  }
  ~AnchorRecorder() {}

  void spin() {

    cv::VideoWriter video( "out.avi", CV_FOURCC('M','J','P','G'), 20, cv::Size( 960/2, 540/2), true);
    //cv::VideoWriter video( "out.avi", CV_FOURCC('P','I','M','1'), 20, cv::Size( 960/2, 540/2), true);
    if ( !video.isOpened() ) {
      cout << "ERROR: Failed to write the video" << endl;
      return;
    }

    ros::Rate rate(30);
    while(ros::ok()) {

      // OpenCV window for display
      if( !this->_img.empty() ) {
	cv::imshow( "Anchors demo...", this->_img );
	if( this->_recording ) {
	  cv::Mat scaled;
	  cv::resize( this->_img, scaled, cv::Size( 960/2, 540/2));
	  video.write(scaled);
	}
      }

      // Wait for a keystroke in the window
      char key = cv::waitKey(1);            
      if( key == 27 || key == 'Q' || key == 'q' ) {
	break;
      }
      else if( key == 'H' || key == 'h' ) {
	this->help();
      }
      else if( key == '1' ) {
	this->triggerPub("segmentation");
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
      else if( key == '2' ) {
	this->triggerPub("tracking");
      }
      else if( key == '3' ) {
	this->triggerPub("classification");
      }
      else if( key == '4' ) {
	this->triggerPub("grounding");
      }
      else if( key == '5' ) {
	this->triggerPub("anchoring");
      }
      else if( key == 'S' || key == 's' ) {
	this->_recording = !this->_recording;
	if( this->_recording ) {
	  std::cout<< "[Start recording]." << std::endl;
	}
	else {
	  std::cout<< "[Stop recording]." << std::endl;
	}
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
  ros::init(argc, argv, "anchor_demo_recorder_node");
  ros::NodeHandle nh;
  AnchorRecorder node(nh);
  node.spin();
  return 0;
}
