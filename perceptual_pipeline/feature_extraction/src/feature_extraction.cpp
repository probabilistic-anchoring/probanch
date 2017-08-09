
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <feature_extraction/feature_extraction.hpp>

//#define USE_KEYPOINT_FEATURES 1

FeatureExtraction::FeatureExtraction(ros::NodeHandle nh) 
  : nh_(nh)
  , priv_nh_("~")
  , it_(nh)
  , display_image_("")
{ 

  // Publisher & subscribers
  obj_sub_ = nh_.subscribe("/objects/raw", 1, &FeatureExtraction::processCb, this);
  obj_pub_ = nh_.advertise<anchor_msgs::ObjectArray>("/objects/processed", 1);

  // Used for the web interface
  display_trigger_sub_ = nh_.subscribe("/display/trigger", 1, &FeatureExtraction::triggerCb, this);
  display_image_pub_ = it_.advertise("/display/image", 1);

  // Load the color classifier
  const std::string path = ros::package::getPath("feature_extraction");
  std::cout << path + "/models/colors.yml" << std::endl; 
  //this->_cf = std::shared_ptr<ColorFeatures>( new ColorFeatures(path + "/models/colors.yml") );
  this->_cf.load(path + "/models/colors.yml");
  std::cout << "Loaded fine..." << std::endl;

} 

void FeatureExtraction::triggerCb( const std_msgs::String::ConstPtr &msg) {
  if( msg->data == "extraction" || msg->data == "grounding" ) {
    this->display_image_ = msg->data;
  }
  else {
    this->display_image_ = "";
  }
}

void FeatureExtraction::processCb(const anchor_msgs::ObjectArray::ConstPtr &objects_msg) {
  // Create a (editable) copy
  anchor_msgs::ObjectArray output;
  output.header = objects_msg->header;
  output.objects = objects_msg->objects;
  output.image = objects_msg->image;
  output.info = objects_msg->info;
  output.transform = objects_msg->transform;

  #ifdef USE_KEYPOINT_FEATURES
  // Instaniate main feature processor
  int numKeyPoints = objects_msg->objects.size() * 2000;
  KeypointFeatures kf(numKeyPoints);
  #endif
  
  // Try to recive the image
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img;
  std::vector<cv::KeyPoint> keypoints;
  try {
    cv_ptr = cv_bridge::toCvCopy( objects_msg->image, 
				  sensor_msgs::image_encodings::BGR8 );
    cv_ptr->image.copyTo(img);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("[FeatureExtraction::process] receiving image: %s", e.what());
    return;
  }

  // Convert to grayscale and back (for display purposes)
  cv::Mat result;
  if( !display_image_.empty() ) {
    cv::cvtColor( img, result, CV_BGR2GRAY); 
    cv::cvtColor( result, result, CV_GRAY2BGR);
    result.convertTo( result, -1, 1.0, 50); 
  }

  // Histogram equalization
  //img = ColorFeatures::equalizeIntensity(img);
  //std::cout << "This part works.. " << std::endl;

  #ifdef USE_KEYPOINT_FEATURES
  // Detect keypoints (for the entire image)
  cv::Mat gray, descriptor;
  cv::cvtColor( img, gray, CV_BGR2GRAY); // <-- Gray scale
  kf.detect( gray, keypoints);
   
  // Extract descriptor
  if (!keypoints.empty()) {
    descriptor = kf.extract( gray, keypoints, numKeyPoints);    
  }
  #endif

  // Go through each contour and extract features for each object
  std::vector< std::vector< cv::KeyPoint> > total_keypoints;
  std::vector<cv::Mat> total_descriptor;
  for (uint i = 0; i < objects_msg->objects.size(); i++) {
  
    // Get the contour
    std::vector<cv::Point> contour;
    for( uint j = 0; j < objects_msg->objects[i].caffe.border.contour.size(); j++) {
      cv::Point p( objects_msg->objects[i].caffe.border.contour[j].x, objects_msg->objects[i].caffe.border.contour[j].y );
      contour.push_back(p);
    }

    // 1. Get keypoint features (from keypoints within the contour)
    // ---------------------------
    #ifdef USE_KEYPOINT_FEATURES
    std::vector<int> idxs;
    std::vector<cv::KeyPoint> sub_keypoints;
    for (uint j = 0; j < keypoints.size(); j++) {
      if( cv::pointPolygonTest( contour, keypoints[j].pt, false ) > 0.0 ) { 
	idxs.push_back(j);
	sub_keypoints.push_back(keypoints[j]);
      } 
    }

    // Filter descriptor (in case we have too many features)
    cv::Mat sub_descriptor = kf.filterDescriptor( descriptor, idxs, CV_8U);
    sub_descriptor = kf.filterResponseN( sub_keypoints, sub_descriptor, 500);

    // Add to total containers
    if( display_image_ == "extraction" ) {
      total_keypoints.push_back(sub_keypoints);
      total_descriptor.push_back(sub_descriptor);
    }
    
    // Add extracted descriptor 
    cv_ptr->image = sub_descriptor;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(output.objects[i].descriptor.data);      
    #endif
    
    // 2. Extrace sub-image (based on the bounding box)
    // --------------------------- 
    cv::Rect rect = cv::boundingRect(contour); 
    rect = rect + cv::Size( 50, 50);  // ...add padding
    rect = rect - cv::Point( 25, 25);
    rect &= cv::Rect( cv::Point(0.0), img.size()); // Saftey routine! 
    cv::Mat sub_img = img(rect);
    sub_img.copyTo(cv_ptr->image); // Skip masking to get full (sub-image) 
    cv_ptr->encoding = "bgr8";
    cv_ptr->toImageMsg(output.objects[i].caffe.data);
    
    // Top-left corner point to message point
    output.objects[i].caffe.point.x = rect.x;
    output.objects[i].caffe.point.y = rect.y; 
    
    // 3. Extract color attribute
    // --------------------------- 

    // Draw the contour image mask 
    cv::Mat mask( img.size(), CV_8U, cv::Scalar(0) );
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    cv::drawContours( mask, contours, -1, cv::Scalar(255), CV_FILLED);

    // Draw masked image on resulting display image
    if( !display_image_.empty() ) {
      img.copyTo( result, mask);
      cv::drawContours( img, contours, -1, cv::Scalar( 0, 0, 255), 1);
    }
    
    // Calculate the HSV color histogram over the image mask
    cv::Mat hist;
    cv::Mat sub_mask = mask(rect);
    this->_cf.calculate( sub_img, hist, sub_mask);

    // Classify the histogram
    std::vector<float> preds;
    this->_cf.predict( hist, preds);
    cv::Mat preds_img( 1, preds.size(), CV_32FC1, &preds.front()); 
    
    /*
    for( int i = 0; i < preds.size(); i++ )
      std::cout << cf_.colorSymbol(i) << ": " << (preds[i] * 100.0) << "%" << std::endl;
    std::cout << " ---- " << std::endl;
    */

    if( display_image_ == "grounding" ) {

      // Draw the contour (for display)
      cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
      //cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
      //cv::Scalar color = cv::Scalar::all(64); // Dark gray
      cv::drawContours( result, contours, -1, color, 1);

      cv::Point2f p1(rect.x, rect.y + 10);
      cv::Point2f p2(rect.x + 105, rect.y + 10);
      cv::line( result, p1, p2, cv::Scalar::all(64), 1, 8);
      p1.x += 7;
      p1.y -= 1;
      for( int i = 0; i < preds.size(); i++ ) {
	if( SHADES_OF_RED_HIGH == i )
	  continue;
	p2.x = p1.x + 6;
	p2.y = p1.y - (int)(preds[i] * 50.0);
	cv::rectangle( result, p1, p2, this->_cf.getColor(i), CV_FILLED);
	p1.x += 7;
      }  
    }

    // Ground color symbols
    double max = *std::max_element( preds.begin(), preds.end());
    for( uint j = 0; j < preds.size(); j++) {
      if( preds[j] / max > 0.75 ) {  // Looking for color spikes above 75 % of the max value
	output.objects[i].color.symbols.push_back(this->_cf.colorSymbol(j));
	output.objects[i].color.predictions.push_back(preds[j]);
      }
    }

    // Create the output "image"
    cv_ptr->image = preds_img;
    cv_ptr->encoding = "32FC1";
    cv_ptr->toImageMsg(output.objects[i].color.data);
    /*
    // Normalize the histogram and add to output (skip masking to get full sub-image)
    cv::Mat hist_reduced;
    cf_.reduce( hist, hist_reduced);
    cf_.normalize(hist_reduced);
    std::vector<cv::Mat> channels;  // Split for handling of multi-channel images 
    cv::split( hist_reduced, channels);
    std::cout << "Number of channels: " << channels.size() << std::endl;
    for( uint j = 0; i < channels.size(); j++) {
      channels[j].copyTo(cv_ptr->image); 
      cv_ptr->encoding = "32FC1";
      output.objects[i].color.data.push_back(*cv_ptr->toImageMsg());
    }
    */
  }

  // Publish the new object array
  obj_pub_.publish(output);
      
  // Publish the resulting feature image
  if( !display_image_.empty() ) {

    if( display_image_ == "extraction"  ) {
    
      for (uint i = 0; i < total_keypoints.size() - 1; i++) {
	if( total_keypoints[i].empty() ) 
	  continue;

	for (uint j = i + 1; j < total_keypoints.size(); j++) {
	  if( total_keypoints[j].empty() ) 
	    continue;
	  /*
	  std::vector<cv::DMatch> matches;
	  kf.match( total_descriptor[i], total_descriptor[j], matches);
	  for( uint k = 0; k < matches.size(); k++ ) {
	    cv::Point2f p1 = total_keypoints[i][matches[k].queryIdx].pt;
	    cv::Point2f p2 = total_keypoints[j][matches[k].trainIdx].pt;
	    cv::line( result, p1, p2, cv::Scalar( 51, 92, 255), 1, 8);
	  }
	  */
	}
      }

#if CV_MAJOR_VERSION == 2 // opencv2 onlt

      for (uint i = 0; i < total_keypoints.size(); i++) {
	if( !total_keypoints[i].empty() )
	  cv::drawKeypoints( result, total_keypoints[i], result, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	  //cv::drawKeypoints( result, total_keypoints[i], result, cv::Scalar( 51, 92, 255), cv::DrawMatchesFlags::DEFAULT);
      }
#endif
      
    }
    cv_ptr->image = result;
    cv_ptr->encoding = "bgr8";
    display_image_pub_.publish(cv_ptr->toImageMsg());
  }

  /*
  // Publish an image with bounding boxes
  cv::Mat resized;
  cv::resize( result, resized, cv::Size(), 0.5, 0.5, CV_INTER_AREA); // Reduce the size
  cv_ptr->image = resized;
  cv_ptr->encoding = "bgr8";
  boxed_pub_.publish(cv_ptr->toImageMsg());
  */
}


void FeatureExtraction::spin() {
  ros::Rate rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}

// ----------------------
// Main function
// -----------------------
int main(int argc, char **argv) {
#if CV_MAJOR_VERSION == 2
  // do opencv 2 code
  std::cout << "Opencv 2" << std::endl;
#elif CV_MAJOR_VERSION == 3
  // do opencv 3 code
  std::cout << "Opencv 3" << std::endl;
#endif
  
  ros::init(argc, argv, "feature_extraction_node");
  ros::NodeHandle nh;
  
  FeatureExtraction node(nh);
  node.spin();
  return 0;
}
