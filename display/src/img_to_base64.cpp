#include <iostream>
#include <vector>
#include <sstream>

//#include <b64/encode.h>
//#include <string>

/* -- Install --
sudo apt-get update
sudo apt-get install libb64-dev
----------- */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <anchor_msgs/DisplayArray.h>

using namespace std;

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

class ImageToBase64 {

  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh; 
  ros::NodeHandle _priv_nh;

  image_transport::ImageTransport _it;
  image_transport::Subscriber _img_sub;
  ros::Publisher _img_str_pub;

  // Encode a string to base64
  std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--) {
      char_array_3[i++] = *(bytes_to_encode++);
      if (i == 3) {
	char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
	char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
	char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
	char_array_4[3] = char_array_3[2] & 0x3f;

	for(i = 0; (i <4) ; i++)
	  ret += base64_chars[char_array_4[i]];
	i = 0;
      }
    }

    if (i)
      {
	for(j = i; j < 3; j++)
	  char_array_3[j] = '\0';

	char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
	char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
	char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
	char_array_4[3] = char_array_3[2] & 0x3f;

	for (j = 0; (j < i + 1); j++)
	  ret += base64_chars[char_array_4[j]];

	while((i++ < 3))
	  ret += '=';
      }
    return ret;
  }

public:
  ImageToBase64(ros::NodeHandle nh) : _nh(nh), _it(nh), _priv_nh("~") {
    
    // ROS subscriber/publisher
    _img_sub = _it.subscribe("/display/image", 10, &ImageToBase64::imageCb, this);
    _img_str_pub = _nh.advertise<std_msgs::String>("/display/base64img", 10);

  }
  ~ImageToBase64() {}

  void imageCb(const sensor_msgs::ImageConstPtr& msg_ptr) {
    
    // Reciive the image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      //cv_ptr = cv_bridge::toCvShare( msg_ptr, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvCopy( msg_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Encode the image to base64
    vector<uchar> buf;
    cv::imencode(".jpg", cv_ptr->image, buf);
    uchar *enc_msg = new uchar[buf.size()];
    for(int i=0; i < buf.size(); i++) { 
      enc_msg[i] = buf[i];
    }
    string encoded = this->base64_encode(enc_msg, buf.size());

    // Send the encoded string
    std_msgs::String msg;
    msg.data = encoded;
    _img_str_pub.publish(msg);
  }

  // ROS handler
  void spin() {
    ros::Rate rate(30);
    while(ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  }

  // For local test
  void spinWebCam() {

    cv::VideoCapture cap(0); // open the default camera
    ros::Rate rate(30);
    while(ros::ok() && cap.isOpened()) {
      cv::Mat img;
      cap >> img; // get a new frame from camera
      //cv::Mat img( 100, 100, CV_8UC3, cv::Scalar(255,0,0));

      // Encode the image to base64
      vector<uchar> buf;
      cv::imencode(".jpg", img, buf);
      uchar *enc_msg = new uchar[buf.size()];
      for(int i=0; i < buf.size(); i++) 
	enc_msg[i] = buf[i];
      string encoded = this->base64_encode(enc_msg, buf.size());

      // Send the encoded string
      std_msgs::String msg;
      msg.data = encoded;
      _img_str_pub.publish(msg);

      ros::spinOnce();
      rate.sleep();
    }
  }
};

// ------------------------------------------
// Main function
// ------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "img_pub_node");
  ros::NodeHandle nh;
  ImageToBase64 node(nh);
  node.spinWebCam();
  return 0;
}
