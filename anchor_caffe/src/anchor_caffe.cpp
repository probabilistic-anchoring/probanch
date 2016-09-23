/*
 * anchor_caffe.cpp
 *
 *  Created on: Nov 21, 2015
 *  Author: Andreas Persson
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <anchor_caffe/CaffeService.h> 
#include <anchor_caffe/classifier.hpp>

#include <anchor_msgs/ObjectArray.h>

using namespace std;

class AnchorCaffe {
  
  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh;
  ros::ServiceServer _caffe_srv;

  // Publisher /subscribers
  ros::Subscriber obj_sub_;
  ros::Publisher obj_pub_;

  // Main classifier object
  Classifier* _classifier;
  int _N;

  // Caffe path variables
  string _model_path;
  string _weights_path;
  string _mean_file;
  string _label_file;
  //string _image_path;


  void process(const anchor_msgs::ObjectArray::ConstPtr &objects_msg) {
    
    // Create a (editable) copy
    anchor_msgs::ObjectArray output;
    output.header = objects_msg->header;
    output.objects = objects_msg->objects;
    output.image = objects_msg->image;


    // Iterate over all objects
    for (uint i = 0; i < objects_msg->objects.size(); i++) {
      
      // Read percept from ROS message
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat img;
      try {
	cv_ptr = cv_bridge::toCvCopy( objects_msg->objects[i].caffe.data,
				      sensor_msgs::image_encodings::BGR8 );
	cv_ptr->image.copyTo(img);
      } catch (cv_bridge::Exception& e) {
	ROS_ERROR("[anchor_cafffe] receiving image: %s", e.what());
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
	//ROS_WARN("[Caffe] object: %s", output.objects[i].caffe.symbols.front().c_str());
      }
    }
    
    // Publish the new object array
    obj_pub_.publish(output);
  
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
      //ROS_WARN("[Caffe] object: %s", res.symbols.front().c_str());
      return true;
    }
    return false;
  }

public:
  AnchorCaffe(ros::NodeHandle nh, int n) : _nh(nh), _N(n) {

    // Get base dir through ros path
    const string ROOT_PATH = ros::package::getPath("anchor_caffe");
    cout << ROOT_PATH<<endl;
    _model_path = ROOT_PATH + "/model/reground.prototxt";
    _weights_path = ROOT_PATH + "/model/finetune_reground.caffemodel";
    _mean_file = ROOT_PATH + "/model/imagenet_mean.binaryproto";
    _label_file = ROOT_PATH + "/model/reground_words.txt";
    //_image_path = ROOT_SAMPLE + "/model/cat.jpg";

    // Create the classifier
    this->_classifier = new Classifier( _model_path, _weights_path, _mean_file, _label_file);

    // ROS setup
    this->_caffe_srv  = _nh.advertiseService("/caffe_classifier", &AnchorCaffe::classify, this);
    obj_sub_ = nh.subscribe("/objects/processed", 1, &AnchorCaffe::process, this);
    obj_pub_ = nh.advertise<anchor_msgs::ObjectArray>("/objects/classified", 1);

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
