
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/String.h>
#include <anchor_msgs/ObjectArray.h>
#include <anchor_msgs/ClusterArray.h>
#include <anchor_msgs/MovementArray.h>

#include <object_segmentation/object_calibration.hpp>

#include <pcl_ros/point_cloud.h>

using namespace std;

// Window name
const char* ObjectCalibration::window_ = "Display window";

// ------------------------
// Public functions
// -------------------------
ObjectCalibration::ObjectCalibration(ros::NodeHandle nh) : 
  nh_ (nh), 
  it_ (nh),
  priv_nh_ ("~"),
  queueSize_ (5),
  plane_size_ (10000),
  cluster_size_ (500),
  angle_th_ (30), 
  distance_th_ (20), 
  factor_ (100) 
{

  // Subscribers / publishers
  image_sub_ = new image_transport::SubscriberFilter( it_, "image", queueSize_);
  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>( nh_, "camera_info", queueSize_);
  cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( nh_, "cloud", queueSize_);
  
  // Set up sync policies
  syncApproximate_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
  syncApproximate_->registerCallback( boost::bind( &ObjectCalibration::segmentationCb, this, _1, _2, _3));

  // Create transformation listener
  tf_listener_ = new tf::TransformListener();
  priv_nh_.param( "base_frame", base_frame_, std::string("base_link"));

  // Read spatial thresholds (default values = no filtering)
  this->priv_nh_.param<double>( "min_x", this->min_x_, 0.0);
  this->priv_nh_.param<double>( "max_x", this->max_x_, -1.0);
  this->priv_nh_.param<double>( "min_y", this->min_y_, 0.0);
  this->priv_nh_.param<double>( "max_y", this->max_y_, -1.0);
  this->priv_nh_.param<double>( "min_z", this->min_z_, 0.0);
  this->priv_nh_.param<double>( "max_z", this->max_z_, -1.0);

  // Add trackbars
  cv::namedWindow( window_, 1);
  cv::createTrackbar( "Plane size:", window_, &this->plane_size_, 100000);
  cv::createTrackbar( "Cluster size:", window_, &this->cluster_size_, 5000);
  cv::createTrackbar( "Angle threshold:", window_, &this->angle_th_, 100);
  cv::createTrackbar( "Distance threshold:", window_, &this->distance_th_, 100);
  cv::createTrackbar( "Refine factor:", window_, &this->factor_, 200);
}
  
ObjectCalibration::~ObjectCalibration() {
  delete syncApproximate_;
  delete image_sub_;
  delete camera_info_sub_;
  delete cloud_sub_;
  delete tf_listener_;
}

void ObjectCalibration::spin() {
  static int counter = 0;
  ros::Rate rate(30);
  while (ros::ok()) {

    if( !this->display_img_.empty() ) {
      cv::imshow( ObjectCalibration::window_, this->display_img_ );
    }
     
    // Wait for a keystroke in the window
    char key = cv::waitKey(1);            
    if( key == 27 || key == 'Q' || key == 'q' ) {
      break;
    }
    else if( key == '0' ) { this->seg_.setComparatorType(0); }
    else if( key == '1' ) { this->seg_.setComparatorType(1); }
    else if( key == '2' ) { this->seg_.setComparatorType(2); }
    else if( key == '3' ) { this->seg_.setComparatorType(3); }

    // Get trackbar values (and sett settings)
    this->seg_.setPlaneMinSize (plane_size_);
    this->seg_.setClusterMinSize (cluster_size_);
    this->seg_.setAngularTh ((double)angle_th_ / 10.0);
    this->seg_.setDistanceTh ((double)distance_th_ / 1000.0);
    this->seg_.setRefineFactor ((double)factor_ / 100.0);

    // Print current settings
    if( counter >= 100 ) {
      ROS_INFO("--[ Settings ]-- ");  
      ROS_INFO("Planar min size: %d", (int)plane_size_);  
      ROS_INFO("Cluster min size: %d", (int)cluster_size_);  
      ROS_INFO("Angle threshold: %.2f", (double)angle_th_ / 10.0);  
      ROS_INFO("Distance threshold: %.3f", (double)distance_th_ / 1000.0);
      ROS_INFO("Refine factor: %.2f", (double)factor_ / 100.0);
      counter = 0;
    }
    counter++;
    ros::spinOnce();
    rate.sleep();
  }    
}

// --------------------------
// Callback function (ROS)
// --------------------------
void ObjectCalibration::segmentationCb( const sensor_msgs::Image::ConstPtr image_msg, 
					const sensor_msgs::CameraInfo::ConstPtr camera_info_msg, 
					const sensor_msgs::PointCloud2::ConstPtr cloud_msg) {
  
  static bool add_once = true;

  // Get the transformation
  tf::StampedTransform transform;
  try{
    tf_listener_->waitForTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.1) ); 
    tf_listener_->lookupTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, transform );
  }
  catch( tf::TransformException ex) {
    ROS_WARN("[ObjectCalibration::callback] %s" , ex.what());
    return;
  }

  // Read the cloud
  pcl::PointCloud<segmentation::Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl::fromROSMsg (*cloud_msg, *raw_cloud_ptr);
  
  // Transform the cloud to the world frame
  pcl::PointCloud<segmentation::Point>::Ptr transformed_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl_ros::transformPointCloud( *raw_cloud_ptr, *transformed_cloud_ptr, transform);       
  
  // Filter the transformed point cloud
  for( auto &p: transformed_cloud_ptr->points) {
    if( !( p.x > this->min_x_ && p.x < this->max_x_ ) ||
	!( p.y > this->min_y_ && p.y < this->max_y_ ) ||
	!( p.z > this->min_z_ && p.z < this->max_z_ ) ) {
      p.x = std::numeric_limits<double>::quiet_NaN(); //infinity();
      p.y = std::numeric_limits<double>::quiet_NaN();
      p.z = std::numeric_limits<double>::quiet_NaN();
      p.b = p.g = p.r = 0;
    }
  }    

  // Read the RGB image
  cv::Mat img, result;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( image_msg, image_msg->encoding);
    cv_ptr->image.copyTo(img);
  }
  catch (cv_bridge::Exception& ex){
    ROS_ERROR("[ObjectSegmentation::callback]: Failed to convert image.");
    return;
  }

  // Saftey check
  if( img.empty() ) {
    return;
  }

  // Convert to grayscale and back (for display purposes)
  cv::cvtColor( img, result, CV_BGR2GRAY); 
  cv::cvtColor( result, result, CV_GRAY2BGR);
  result.convertTo( result, -1, 1.0, 50); 

  // Cluster cloud into objects 
  // ----------------------------------------
  std::vector<pcl::PointIndices> cluster_indices;
  this->seg_.clusterOrganized(transformed_cloud_ptr, cluster_indices);
  if( !cluster_indices.empty() ) {
  
    // Camera information
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info_msg);

    // Process the segmented clusters
    for (size_t i = 0; i < cluster_indices.size (); i++) {
      
      // Create the point cluster from the orignal point cloud
      pcl::PointCloud<segmentation::Point>::Ptr cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl::copyPointCloud( *raw_cloud_ptr, cluster_indices[i], *cluster_ptr);
  
      // Pots-process the cluster
      try {

	// Draw the contour (for display)
	cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
	//cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
	//cv::Scalar color = cv::Scalar::all(64); // Dark gray

	// Create the cluster image 
	cv::Mat cluster_img( img.size(), CV_8U, cv::Scalar(0));
	BOOST_FOREACH  ( const segmentation::Point pt, cluster_ptr->points ) {
	  cv::Point3d pt_cv( pt.x, pt.y, pt.z);
	  cv::Point2f p = cam_model.project3dToPixel(pt_cv);
	  circle( result, p, 1, color, -1, 8, 0 );
	}

      }
      catch( cv::Exception &exc ) {
	ROS_ERROR("[object_recognition] CV processing error: %s", exc.what() );
      }
    }    
  }
  result.copyTo(this->display_img_);
}

// ----------------------
// Main function
// ------------------
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "object_segmentation_node");
  ros::NodeHandle nh;

  // Saftey check
  if(!ros::ok()) {
    return 0;
  }

  ObjectCalibration node(nh);
  node.spin();
  
  ros::shutdown();
  return 0;
}
