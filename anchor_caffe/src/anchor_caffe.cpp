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
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <anchor_caffe/CaffeService.h> 
#include <anchor_caffe/classifier.hpp>

#include <std_msgs/String.h>
#include <anchor_msgs/ObjectArray.h>

using namespace std;

class AnchorCaffe {
  
  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh;
  ros::ServiceServer _caffe_srv;
  image_transport::ImageTransport _it;

  // Publisher /subscribers
  ros::Subscriber obj_sub_;
  ros::Publisher obj_pub_;

  bool display_image_;
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
  //string _image_path;

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
    cv::Mat img, result;
    if( this->display_image_ ) {
      try {
	cv_ptr = cv_bridge::toCvCopy( objects_msg->image, 
				      sensor_msgs::image_encodings::BGR8 );
	cv_ptr->image.copyTo(img);

	// Convert to gray (and back to BGR again) 
	cv::cvtColor( img, result, CV_BGR2GRAY); 
	cv::cvtColor( result, result, CV_GRAY2BGR);
	result.convertTo( result, -1, 1.0, 50); 
	
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("[AnchorCaffe::process] receiving image: %s", e.what());
	return;
      }
    }

    // Iterate over all objects
    for (uint i = 0; i < objects_msg->objects.size(); i++) {
      
      // Read percept from ROS message
      cv::Mat img;
      try {
	cv_ptr = cv_bridge::toCvCopy( objects_msg->objects[i].caffe.data,
				      sensor_msgs::image_encodings::BGR8 );
	cv_ptr->image.copyTo(img);
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("[AnchorCaffe::process] receiving image: %s", e.what());
	return;
      }    

      // Classify image
      if( img.data ) {
	vector<Prediction> predictions = this->_classifier->Classify(img, this->_N);
	vector<Prediction>::iterator ite = predictions.begin();
	for( ; ite != predictions.end(); ++ite ) {
	  string str = ite->first;
	  size_t pos = str.find(' ');
	  str.substr( 0, pos);
	  pos++;
	  output.objects[i].caffe.symbols.push_back(str.substr(pos));
	  output.objects[i].caffe.predictions.push_back(ite->second);
	}
	ROS_INFO("[Caffe] object: %s (%.2f)", output.objects[i].caffe.symbols.front().c_str(),
		 output.objects[i].caffe.predictions.front());
      }

      // Draw the result
      if( this->display_image_ && img.data ) {

	cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
	//cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
	//cv::Scalar color = cv::Scalar::all(64); // Dark gray

	int x = objects_msg->objects[i].caffe.point.x;
	int y = objects_msg->objects[i].caffe.point.y;
	cv::Rect rect( cv::Point(x,y), img.size());
	img.copyTo(result(rect));
	
	cv::rectangle( result, rect, color, 1);

	std::stringstream ss;
	ss << setprecision(2) << fixed;
	int offset = -42;
	for( uint j = 0; j < 3; j++) {
	  ss << "#" << (j+1) << ": " << output.objects[i].caffe.symbols[j];
	  ss << " (" << output.objects[i].caffe.predictions[j] * 100.0 << "%)";
	  cv::putText( result, ss.str(), cv::Point( rect.x, rect.y + offset), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	  ss.str("");
	  offset += 16;
	}
      }
    }
    
    // Publish the new object array
    obj_pub_.publish(output);

    // Publish the resulting feature image
    if( display_image_ ) {
      cv_ptr->image = result;
      cv_ptr->encoding = "bgr8";
      display_image_pub_.publish(cv_ptr->toImageMsg());
    }
  }


  // Private callback function for receiving and classifying an image
  bool classify( anchor_caffe::CaffeService::Request &req,
		 anchor_caffe::CaffeService::Response &res ) {
    
    // Read percept from ROS message
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img;
    try {
      cv_ptr = cv_bridge::toCvCopy( req.image,
				    sensor_msgs::image_encodings::BGR8 );
      cv_ptr->image.copyTo(img);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("[anchor_cafffe] receiving image: %s", e.what());
      return false;
    }    

    // Classify image
    if( img.data ) {
      vector<Prediction> predictions = this->_classifier->Classify(img, this->_N);
      vector<Prediction>::iterator ite = predictions.begin();
      for( ; ite != predictions.end(); ++ite ) {
	string str = ite->first;
	size_t pos = str.find(' ');
	res.wordnet_ids.push_back(str.substr( 0, pos));
	pos++;
	res.symbols.push_back(str.substr(pos));
	res.predictions.push_back(ite->second);
      }
      ROS_WARN("[Caffe] object: %s (%.2f)", res.symbols.front().c_str(), res.predictions.front());
      return true;
    }
    return false;
  }

public:
  AnchorCaffe(ros::NodeHandle nh, int n) : _nh(nh), _N(n), _it(nh), display_image_(false) {

    // Get base dir through ros path
    const string ROOT_PATH = ros::package::getPath("anchor_caffe");
    cout << ROOT_PATH<<endl;
    _model_path = ROOT_PATH + "/model/reground.prototxt";
    _weights_path = ROOT_PATH + "/model/reground_googlenet.caffemodel";
    //_weights_path = ROOT_PATH + "/model/finetune_reground.caffemodel";
    //_mean_file = ROOT_PATH + "/model/imagenet_mean.binaryproto";
    _label_file = ROOT_PATH + "/model/reground_words.txt";
    //_image_path = ROOT_SAMPLE + "/model/cat.jpg";

    // Create the classifier
    //this->_classifier = new Classifier( _model_path, _weights_path, _label_file, _mean_file);
    this->_classifier = new Classifier( _model_path, _weights_path, _label_file);

    // ROS setup
    this->_caffe_srv  = _nh.advertiseService("/caffe_classifier", &AnchorCaffe::classify, this);
    obj_sub_ = nh.subscribe("/objects/processed", 1, &AnchorCaffe::processCb, this);
    obj_pub_ = nh.advertise<anchor_msgs::ObjectArray>("/objects/classified", 1);

    // Used for the web interface
    display_trigger_sub_ = _nh.subscribe("/display/trigger", 1, &AnchorCaffe::triggerCb, this);
    display_image_pub_ = _it.advertise("/display/image", 1);

  };

  ~AnchorCaffe() {
    // Clean up...
    delete this->_classifier; 
  }

  // For 'ROS loop' access
  void spin() {
    ros::Rate rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

// -----------------------------------------
// Main function
// -----------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "anchor_caffe_node");

  // Read the number of predictions to use
  int n;
  if( !ros::param::get("~predictions", n) ) {
    n = 5;  // Defualt: 5
  }

  ros::NodeHandle nh;
  AnchorCaffe node(nh, n);
  node.spin();
  return 0;
}
// ------------------------------
