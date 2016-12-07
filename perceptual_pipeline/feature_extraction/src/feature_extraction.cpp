#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <feature_extraction/feature_extraction.hpp>

FeatureExtraction::FeatureExtraction(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), it_(nh) { 

  // Publisher & subscribers
  obj_sub_ = nh_.subscribe("/objects/raw", 1, &FeatureExtraction::process, this);
  obj_pub_ = nh_.advertise<anchor_msgs::ObjectArray>("/objects/processed", 1);
  boxed_pub_ = it_.advertise("/display/boxed", 1);
} 

void FeatureExtraction::process(const anchor_msgs::ObjectArray::ConstPtr &objects_msg) {

  // Create a (editable) copy
  anchor_msgs::ObjectArray output;
  output.header = objects_msg->header;
  output.objects = objects_msg->objects;
  output.image = objects_msg->image;

  // Instaniate main feature processor
  int numKeyPoints = objects_msg->objects.size() * 2000;
  KeypointFeatures kf(numKeyPoints);

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

  // Histogram equalization
  img = ColorFeatures::equalizeIntensity(img);
  std::cout << "This part works.. " << std::endl;
  
  // Detect keypoints (for the entire image)
  cv::Mat gray, descriptor;
  cv::cvtColor( img, gray, CV_BGR2GRAY); // <-- Gray scale
  kf.detect( gray, keypoints);
   
  // Extract descriptor
  if (!keypoints.empty()) {
    descriptor = kf.extract( gray, keypoints, numKeyPoints);    
  }

  // Go through each contour and extract features for each object
  cv::Mat result;
  img.copyTo(result);
  for (uint i = 0; i < objects_msg->objects.size(); i++) {
  
    // Get the contour
    std::vector<cv::Point> contour;
    for( uint j = 0; j < objects_msg->objects[i].caffe.border.contour.size(); j++) {
      cv::Point p( objects_msg->objects[i].caffe.border.contour[j].x, objects_msg->objects[i].caffe.border.contour[j].y );
      contour.push_back(p);
    }

    
    // 1. Get keypoint features (from keypoints within the contour)
    // ---------------------------
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
    sub_descriptor = kf.filterResponseN( sub_keypoints, sub_descriptor, 2000);
    
    // Add extracted descriptor 
    cv_ptr->image = sub_descriptor;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(output.objects[i].descriptor.data);      
    
    
    // 2. Extrace sub-image (based on the bounding box)
    // --------------------------- 
    cv::Rect rect = cv::boundingRect(contour); 
    rect = rect + cv::Size( 100, 100);  // ...add padding
    rect = rect - cv::Point( 50, 50);
    rect &= cv::Rect( cv::Point(0.0), img.size()); // Saftey routine! 
    cv::Mat sub_img = img(rect);
    sub_img.copyTo(cv_ptr->image); // Skip masking to get full (sub-image) 
    cv_ptr->encoding = "bgr8";
    cv_ptr->toImageMsg(output.objects[i].caffe.data);
    
    // Draw rect on resulting display image
    cv::rectangle( result, rect, cv::Scalar( 0, 0, 255), 2);

    // Top-left corner point to message point
    output.objects[i].caffe.point.x = rect.x;
    output.objects[i].caffe.point.y = rect.y; 
    
    // 3. Extract color attribute
    // --------------------------- 

    // Draw the contour image mask 
    cv::Mat mask( img.size(), CV_8U, cv::Scalar(0) );
    std::vector<std::vector<cv::Point> > contours;
    contours.push_back(contour);
    cv::drawContours( mask, contours, -1, cv::Scalar(255), -1);
    
    // Calculate the HSV color histogram over the image mask
    cv::Mat hist;
    cv::Mat sub_mask = mask(rect);
    cf_.calculate( sub_img, hist, sub_mask);
    
    // Classify the histogram
    std::vector<float> preds;
    cf_.predict( hist, preds);
    cv::Mat preds_img( 1, preds.size(), CV_32FC1, &preds.front()); 
    /*
    for( int i = 0; i < preds.size(); i++ )
      std::cout << cf_.colorSymbol(i) << ": " << (preds[i] * 100.0) << "%" << std::endl;
    std::cout << " ---- " << std::endl;
    */

    // Ground color symbols
    double min, max;
    cv::minMaxLoc( preds_img, &min, &max);
    for( uint j = 0; j < preds.size(); j++) {
      if( preds[j] / max > 0.75 ) {  // Looking for color spikes above 75 % of the best spike
	output.objects[i].color.symbols.push_back(cf_.colorSymbol(j));
	output.objects[i].color.predictions.push_back(preds[j]);
      }
    }

    // Normalize the histogram and add to output
    //cf_.normalize(hist);
    preds_img.copyTo(cv_ptr->image); // Skip masking to get full (sub-image) 
    cv_ptr->encoding = "32FC1";
    cv_ptr->toImageMsg(output.objects[i].color.data);
  }

  // Publish the new object array
  obj_pub_.publish(output);
      
  // Publish an image with bounding boxes
  cv::Mat resized;
  cv::resize( result, resized, cv::Size(), 0.5, 0.5, CV_INTER_AREA); // Reduce the size
  cv_ptr->image = resized;
  cv_ptr->encoding = "bgr8";
  boxed_pub_.publish(cv_ptr->toImageMsg());
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
  ros::init(argc, argv, "feature_extraction_node");
  ros::NodeHandle nh;

  FeatureExtraction node(nh);
  node.spin();
  return 0;
}
