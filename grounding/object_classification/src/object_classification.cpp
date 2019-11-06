/*
 * anchor_caffe.cpp
 *
 *  Created on: Nov 21, 2015
 *  Author: Andreas Persson
 */

#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Switch between Caffe and OpenCV implementation (defined in CMakeList)
#ifdef INCLUDE_CAFFE_CLASSIFIER
  #include <object_classification/caffe_classifier.hpp>
#else
  #include <object_classification/cv_classifier.hpp>
#endif

#include <std_msgs/String.h>
#include <object_classification/CaffeService.h>
#include <anchor_msgs/ObjectArray.h>

using namespace std;

class ObjectClassification {
  
  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh;
  ros::ServiceServer _caffe_srv;
  image_transport::ImageTransport _it;

  // Publisher /subscribers
  ros::Subscriber obj_sub_;
  ros::Publisher obj_pub_;

  bool display_image_, display_window_;
  ros::Subscriber display_trigger_sub_;
  image_transport::Publisher display_image_pub_;

  // Main classifier object
  Classifier* _classifier;
  int _N;

  // Caffe path variables
  string _model_path;
  string _weights_path;
  string _mean_file;
  string _label_file;

  // For displaying a streaming result image
  cv::Mat result_img_;
  
  void triggerCb( const std_msgs::String::ConstPtr &msg) {
    this->display_image_ = (msg->data == "classification") ? true : false;
  }

  void processCb(const anchor_msgs::ObjectArray::ConstPtr &objects_msg) {
    
    // Create a (editable) copy
    anchor_msgs::ObjectArray output;
    output.header = objects_msg->header;
    output.objects = objects_msg->objects;
    output.image = objects_msg->image;
    output.info = objects_msg->info;
    output.transform = objects_msg->transform;
    
    // Red the raw image
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    if( this->display_image_ || display_window_) {
      try {
	cv_ptr = cv_bridge::toCvCopy( objects_msg->image, 
				      sensor_msgs::image_encodings::BGR8 );
	cv_ptr->image.copyTo(img);

	// Convert to gray (and back to BGR again)
	cv::cvtColor( img, result_img_, CV_BGR2GRAY); 
	cv::cvtColor( result_img_, result_img_, CV_GRAY2BGR);
	result_img_.convertTo( result_img_, -1, 1.0, 50); 
	
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("[ObjectClassification::process] receiving image: %s", e.what());
	return;
      }
    }

    // Iterate over all objects
    for (uint i = 0; i < objects_msg->objects.size(); i++) {
      
      // Read percept from ROS message
      cv::Mat img;
      try {
	cv_ptr = cv_bridge::toCvCopy( objects_msg->objects[i].visual.data,
				      sensor_msgs::image_encodings::BGR8 );
	cv_ptr->image.copyTo(img);
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("[ObjectClassification::process] receiving image: %s", e.what());
	return;
      }    

      // Classify image
      //auto t = cv::getTickCount();
      if( img.data ) {
	vector<Prediction> predictions = this->_classifier->Classify(img);
	vector<Prediction>::iterator ite = predictions.begin();
	for( ; ite != predictions.end(); ++ite ) {
	  string str = ite->first;
	  size_t pos = str.find(' ');
	  str.substr( 0, pos);
	  pos++;
	  output.objects[i].category.symbols.push_back(str.substr(pos));
	  output.objects[i].category.predictions.push_back(ite->second);
	}
      }
      //double s = (cv::getTickCount() - t) / cv::getTickFrequency();
      //ROS_INFO("Classification time: %.2f (ms)", (100.0 * s));
	       
      // Draw the result
      if( (this->display_image_ || this->display_window_ ) && img.data ) {

	cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
	//cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
	//cv::Scalar color = cv::Scalar::all(64); // Dark gray

	int x = objects_msg->objects[i].visual.point.x;
	int y = objects_msg->objects[i].visual.point.y;
	cv::Rect rect( cv::Point(x,y), img.size());
	img.copyTo(result_img_(rect));
	
	cv::rectangle( result_img_, rect, color, 1);
	
	std::stringstream ss;
	ss << setprecision(2) << fixed;
	int offset = -10 - (16 * (this->_N - 1));
	for( uint j = 0; j < this->_N; j++) {
	  ss << (j+1) << ". " << output.objects[i].category.symbols[j];
	  ss << " (" << output.objects[i].category.predictions[j] * 100.0 << "%)";
	  cv::putText( result_img_, ss.str(), cv::Point( rect.x, rect.y + offset), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	  ss.str("");
	  offset += 16;
	}
	
	/*
	// TMP
	std::stringstream ss;
	ss << setprecision(2) << fixed;
	int offset = -6;
	for( uint j = 0; j < 1; j++) {
	  ss << output.objects[i].caffe.symbols[j];
	  ss << " (" << output.objects[i].caffe.predictions[j] * 100.0 << "%)";
	  cv::putText( result_img_, ss.str(), cv::Point( rect.x, rect.y + offset), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	  ss.str("");
	  offset += 16;
	}
	*/
      }
    }

    // Publish the new object array
    obj_pub_.publish(output);

    // Publish the resulting feature image
    if( display_image_ ) {
      cv_ptr->image = this->result_img_;
      cv_ptr->encoding = "bgr8";
      display_image_pub_.publish(cv_ptr->toImageMsg());
    }
  }

public:
  ObjectClassification(ros::NodeHandle nh, int n, bool display) : _nh(nh), _N(n), _it(nh), display_window_(display), display_image_(false) {

    // Get base dir through ros path
    const string ROOT_PATH = ros::package::getPath("object_classification");
    cout << ROOT_PATH<<endl;
    _model_path = ROOT_PATH + "/model/reground.prototxt";
    _weights_path = ROOT_PATH + "/model/reground_googlenet.caffemodel";
    _label_file = ROOT_PATH + "/model/reground_words.txt";

    // Create the classifier
    this->_classifier = new Classifier( _model_path, _weights_path, _label_file);

    // ROS setup
    obj_sub_ = nh.subscribe("/objects/processed", 10, &ObjectClassification::processCb, this);
    obj_pub_ = nh.advertise<anchor_msgs::ObjectArray>("/objects/classified", 10);

    // Used for the web interface
    display_trigger_sub_ = _nh.subscribe("/display/trigger", 1, &ObjectClassification::triggerCb, this);
    display_image_pub_ = _it.advertise("/display/image", 1);

  };

  ~ObjectClassification() {
    // Clean up...
    delete this->_classifier; 
  }

  // For 'ROS loop' access
  void spin() {
    ros::Rate rate(30);
    while (ros::ok()) {

      
      // OpenCV window for display
      if( this->display_window_ && !this->result_img_.empty() ) {
	cv::imshow( "Classified objects...", this->result_img_ );
      
	// Wait for a keystroke in the window
	char key = cv::waitKey(1);            
	if( key == 27 || key == 'Q' || key == 'q' ) {
	  break;
	}
      }
      
      ros::spinOnce();
      rate.sleep();
    }
  }
};

// -----------------------------------------
// Main function
// -----------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "object_classification_node");

  // Read the number of top predictions to display
  int n;
  if( !ros::param::get("~top_predictions", n) ) {
    n = 5;  // Defualt: 5
  }

  // Read toggle for diplaying a result window
  bool display;
  if( !ros::param::get("~display_window", display) ) {
    display = false;  // Defualt: false
  }  

  ros::NodeHandle nh;
  ObjectClassification node(nh, n, display);
  node.spin();
  return 0;
}
// ------------------------------
