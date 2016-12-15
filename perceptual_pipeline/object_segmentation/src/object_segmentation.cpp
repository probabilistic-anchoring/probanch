
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

#include <object_segmentation/object_segmentation.hpp>

// ------------------------
// Public functions
// -------------------------
ObjectSegmentation::ObjectSegmentation(ros::NodeHandle nh, bool useApprox) 
  : nh_(nh)
  , useApprox_(useApprox)
  , it_(nh)
  , priv_nh_("~")
  , queueSize_(5)
  , display_image_(false) {

  // Subscribers / publishers
  //image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
  //image_sub_ = new image_transport::SubscriberFilter( it_, topicColor, "image", hints);

  image_sub_ = new image_transport::SubscriberFilter( it_, "image", queueSize_);
  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>( nh_, "camera_info", queueSize_);
  cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( nh_, "cloud", queueSize_);
  
  obj_pub_ = nh_.advertise<anchor_msgs::ObjectArray>("/objects/raw", queueSize_);
  cluster_pub_ = nh_.advertise<anchor_msgs::ClusterArray>("/objects/clusters", queueSize_);

  // Used for the web interface
  display_trigger_sub_ = nh_.subscribe("/display/trigger", 1, &ObjectSegmentation::triggerCb, this);
  display_image_pub_ = it_.advertise("/display/image", 1);
  
  // Set up sync policies
  if(useApprox) {
    syncApproximate_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
    syncApproximate_->registerCallback( boost::bind( &ObjectSegmentation::segmentationCb, this, _1, _2, _3));
  }
  else {
    syncExact_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
    syncExact_->registerCallback( boost::bind( &ObjectSegmentation::segmentationCb, this, _1, _2, _3));
  }

  // Create transformation listener
  tf_listener_ = new tf::TransformListener();
  priv_nh_.param( "base_frame", base_frame_, std::string("base_link"));

  // Read the compare type for to organized segmentation
  this->priv_nh_.param( "compar_type", this->type_, 3);
}
  
ObjectSegmentation::~ObjectSegmentation() {
  if(useApprox_) {
    delete syncApproximate_;
  }
  else {
    delete syncExact_;
  }
  delete image_sub_;
  delete camera_info_sub_;
  delete cloud_sub_;
  delete tf_listener_;
}

void ObjectSegmentation::spin() {
  while (ros::ok()) {
    ros::spin();
  }    
}

void ObjectSegmentation::triggerCb( const std_msgs::String::ConstPtr &msg) {
  this->display_image_ = (msg->data == "segmentation") ? true : false;
  ROS_WARN("Got trigger: %s", msg->data.c_str()); 
}


// --------------------------
// Callback function (ROS)
// --------------------------
void ObjectSegmentation::segmentationCb( const sensor_msgs::Image::ConstPtr image_msg, 
					 const sensor_msgs::CameraInfo::ConstPtr camera_info_msg, 
					 const sensor_msgs::PointCloud2::ConstPtr cloud_msg) {
  
  // Get the transformation
  tf::StampedTransform transform;
  try{
    tf_listener_->waitForTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.5) ); 
    tf_listener_->lookupTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, transform );
  }
  catch( tf::TransformException ex) {
    ROS_WARN("[ObjectSegmentation::callback] %s" , ex.what());
    return;
  }

  // Read the cloud
  pcl::PointCloud<segmentation::Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl::fromROSMsg (*cloud_msg, *raw_cloud_ptr);

  /*
  // Transform the cloud to the world frame
  pcl::PointCloud<segmentation::Point>::Ptr transformed_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl_ros::transformPointCloud( *raw_cloud_ptr, *transformed_cloud_ptr, transform);       
 
  // Filter the cloud (to reduce the size)
  segmentation::passThroughFilter( transformed_cloud_ptr, transformed_cloud_ptr, "x", 0.2, 1.2 );
  segmentation::passThroughFilter( transformed_cloud_ptr, transformed_cloud_ptr, "y", 0.0, 0.62 );
  segmentation::passThroughFilter( transformed_cloud_ptr, transformed_cloud_ptr, "z", -0.1, 0.6 ); 
  */

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
  if( display_image_ ) {
    cv::cvtColor( img, result, CV_BGR2GRAY); 
    cv::cvtColor( result, result, CV_GRAY2BGR);
    result.convertTo( result, -1, 1.0, 50); 
  }

  // Cluster cloud into objects 
  // ----------------------------------------
  segmentation::Segmentation seg(raw_cloud_ptr);
  std::vector<pcl::PointIndices> cluster_indices;
  seg.cluster_organized(cluster_indices, this->type_);
  if( !cluster_indices.empty() ) {
  
    // Camera information
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info_msg);

    // Image & Cloud output
    anchor_msgs::ClusterArray clusters;
    anchor_msgs::ObjectArray objects;
    objects.header = cloud_msg->header;
    objects.image = *image_msg;

    // Process the segmented clusters
    for (size_t i = 0; i < cluster_indices.size (); i++) {
      
      // Create the point cluster
      pcl::PointCloud<segmentation::Point>::Ptr cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl::copyPointCloud( *raw_cloud_ptr, cluster_indices[i], *cluster_ptr);
  
      // Pots-process the cluster
      //std::vector<pcl::Vertices> indices;
      //seg.post_process( cluster_ptr, indices);
      try {

	// Create the cluster image 
	cv::Mat cluster_img( img.size(), CV_8U, cv::Scalar(0));
	BOOST_FOREACH  ( const segmentation::Point pt, cluster_ptr->points ) {
	  cv::Point3d pt_cv( pt.x, pt.y, pt.z);
	  cv::Point2f p = cam_model.project3dToPixel(pt_cv);
	  circle( cluster_img, p, 1, cv::Scalar(255), -1, 8, 0 );
	}
	/*
	for (size_t j = 0; j < indices.size (); j++) {
	  std::vector<cv::Point> poly;
	  for (size_t k = 0; k < indices[j].vertices.size(); k++) {
	    segmentation::Point p = cluster_ptr->at(indices[j].vertices[k]);
	    cv::Point3d pt_cv( p.x, p.y, p.z);
	    poly.push_back( cam_model.project3dToPixel(pt_cv) );
	  }
	  const cv::Point *poly_ptr = &poly[0];
	  cv::fillConvexPoly( cluster_img, poly_ptr, (int)poly.size(), cv::Scalar(255));
	}
	*/

	// Find the contours of the cluster
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( cluster_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	// Create and add the object output message
	if( contours.size() > 0 ) {

	  anchor_msgs::Object obj;
	  
	  for( size_t j = 0; j < contours[0].size(); j++) {  
	    anchor_msgs::Point2d p;
	    p.x = contours[0][j].x;
	    p.y = contours[0][j].y;
	    obj.caffe.border.contour.push_back(p);
	  }
	  
	  /*
	  // Draw the contour (for display)
	  cv::drawContours( img, contours, -1, cv::Scalar( 0, 0, 255), 2);
      	  */
	  
	  // Transform the cloud to the world frame
	  pcl::PointCloud<segmentation::Point> transformed_cluster;
	  pcl_ros::transformPointCloud( *cluster_ptr, transformed_cluster, transform);
	  

	  // 1. Extract the location
	  obj.location.data.header.stamp = cloud_msg->header.stamp;
	  //segmentation::getLocation( cluster_ptr, obj.location.data.pose );
	  segmentation::getLocation( transformed_cluster.makeShared(), obj.location.data.pose );
	  if( !( obj.location.data.pose.position.x > 0.2  && obj.location.data.pose.position.y > 0.0 && obj.location.data.pose.position.y < 0.62 && obj.location.data.pose.position.z < 0.5 ) )
	    continue;

	  // Add segmented object to display image
	  img.copyTo( result, cluster_img);
	  

	  // std::cout << "Location: [" << obj.location.data.pose.position.x << ", " << obj.location.data.pose.position.y << ", " << obj.location.data.pose.position.z << "]" << std::endl;	  

	  // 2. Extract the shape
	  //segmentation::getShape( cluster_ptr, obj.shape.data );
	  segmentation::getShape( transformed_cluster.makeShared(), obj.shape.data );

	  // Ground shape symbols
	  std::vector<double> data = { obj.shape.data.x, obj.shape.data.y, obj.shape.data.z};
	  std::sort( data.begin(), data.end(), std::greater<double>());
	  if( data.front() <= 0.1 ) { // 0 - 15 [cm] = small
	    obj.shape.symbols.push_back("small");
	  }
	  else if( data.front() <= 0.20 ) { // 16 - 30 [cm] = medium
	    obj.shape.symbols.push_back("medium");
	  }
	  else {  // > 30 [cm] = large
	    obj.shape.symbols.push_back("large");
	  }
	  if( data[0] < data[1] * 1.1 ) {
	    obj.shape.symbols.push_back("square");
	  }
	  else {
	    obj.shape.symbols.push_back("rectangle");
	    if( data[0] > data[1] * 1.5 ) {
	      obj.shape.symbols.push_back("long");
	    }
	    else {
	      obj.shape.symbols.push_back("short");
	    }
	  }
	  
	  // Add the object to the object array message
	  objects.objects.push_back(obj);

	  /*
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid ( transformed_cloud, centroid);

	  if( centroid[0] > 0.0 && centroid[1] < 0.5 && centroid[1] > -0.5 )
	    std::cout << "Location: [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "]" << std::endl;
	  */
	  
	  // Add the (un-transformed) cluster to the array message
	  geometry_msgs::Pose center;
	  segmentation::getLocation( cluster_ptr, center );
	  clusters.centers.push_back(center);

	  sensor_msgs::PointCloud2 cluster;
	  pcl::toROSMsg( *cluster_ptr, cluster );
	  clusters.clusters.push_back(cluster);
	  
	}
      }
      catch( cv::Exception &exc ) {
	ROS_ERROR("[object_recognition] CV processing error: %s", exc.what() );
      }
    }

    // Publish the "segmented" image
    if( display_image_ ) {
      cv_ptr->image = result;
      cv_ptr->encoding = "bgr8";
      display_image_pub_.publish(cv_ptr->toImageMsg());
    }
    
    // Publish the object array
    if( !objects.objects.empty() || !clusters.clusters.empty() ) {
      //ROS_INFO("Publishing: %d objects.", (int)objects.objects.size());
      obj_pub_.publish(objects);
      cluster_pub_.publish(clusters);
    }
    
  }
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

  ObjectSegmentation node(nh);
  node.spin();
  
  ros::shutdown();
  return 0;
}
