
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

#include <object_segmentation/object_segmentation.hpp>

#include <pcl_ros/point_cloud.h>

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
  move_pub_ = nh_.advertise<anchor_msgs::MovementArray>("/movements", queueSize_);

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

  // Read segmentation parameters
  int type, size;
  double th, factor;
  if( priv_nh_.getParam("compare_type", type) ) {
    if( type >= 0 && type <= 3 ) {
      this->seg_.setComparatorType(type);
    }
    else {
      ROS_WARN("[ObjectSegmentation::ObjectSegmentation] Not a valid comparator type, using default instead.");
    }
  }
  if( priv_nh_.getParam("plane_min_size", size) ) {
    this->seg_.setPlaneMinSize (size);
  }
  if( priv_nh_.getParam("cluster_min_size", size) ) {
    this->seg_.setClusterMinSize (size);
  }
  if( priv_nh_.getParam("cluster_min_size", size) ) {
    this->seg_.setClusterMinSize (size);
  }
  if( priv_nh_.getParam("angular_th", th) ) {
    this->seg_.setAngularTh (th);
  }
  if( priv_nh_.getParam("distance_th", th) ) {
    this->seg_.setDistanceTh (th);
  }
  if( priv_nh_.getParam("refine_factor", factor) ) {
    this->seg_.setRefineFactor (factor);
  }

  // Read spatial thresholds (default values = no filtering)
  this->priv_nh_.param<double>( "min_x", this->min_x_, 0.0);
  this->priv_nh_.param<double>( "max_x", this->max_x_, -1.0);
  this->priv_nh_.param<double>( "min_y", this->min_y_, 0.0);
  this->priv_nh_.param<double>( "max_y", this->max_y_, -1.0);
  this->priv_nh_.param<double>( "min_z", this->min_z_, 0.0);
  this->priv_nh_.param<double>( "max_z", this->max_z_, -1.0);
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
    tf_listener_->waitForTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.01) ); 
    tf_listener_->lookupTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, transform );
  }
  catch( tf::TransformException ex) {
    ROS_WARN("[ObjectSegmentation::callback] %s" , ex.what());
    return;
  }

  // Read the cloud
  pcl::PointCloud<segmentation::Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl::fromROSMsg (*cloud_msg, *raw_cloud_ptr);

  
  // Transform the cloud to the world frame
  pcl::PointCloud<segmentation::Point>::Ptr transformed_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl_ros::transformPointCloud( *raw_cloud_ptr, *transformed_cloud_ptr, transform);       

  // Filter the transformed point cloud 
  this->filter (transformed_cloud_ptr);
  // ----------------------
  
  
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
  std::vector<pcl::PointIndices> cluster_indices;
  this->seg_.clusterOrganized(transformed_cloud_ptr, cluster_indices);
  //this->seg_.clusterOrganized(raw_cloud_ptr, cluster_indices);
  if( !cluster_indices.empty() ) {
  
    // Camera information
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info_msg);

    // Image & Cloud output
    anchor_msgs::ClusterArray clusters;
    anchor_msgs::ObjectArray objects;
    anchor_msgs::MovementArray movements;
    objects.header = cloud_msg->header;
    objects.image = *image_msg;
    objects.info = *camera_info_msg;

    // Store the inverse transformation
    tf::Quaternion tf_quat = transform.inverse().getRotation();
    objects.transform.rotation.x = tf_quat.x();
    objects.transform.rotation.y = tf_quat.y();
    objects.transform.rotation.z = tf_quat.z();
    objects.transform.rotation.w = tf_quat.w();

    tf::Vector3 tf_vec = transform.inverse().getOrigin();
    objects.transform.translation.x = tf_vec.getX();
    objects.transform.translation.y = tf_vec.getY();
    objects.transform.translation.z = tf_vec.getZ();
 
    // Process the segmented clusters
    for (size_t i = 0; i < cluster_indices.size (); i++) {
      
      pcl::PointCloud<segmentation::Point>::Ptr transformed_cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl::copyPointCloud( *transformed_cloud_ptr, cluster_indices[i], *transformed_cluster_ptr);
      if( transformed_cluster_ptr->points.empty() )
	continue;
      
      
      // Reverser transform to get the contour right
      pcl::PointCloud<segmentation::Point>::Ptr cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl_ros::transformPointCloud( *transformed_cluster_ptr, *cluster_ptr, transform.inverse());       

      /*
      // Create the point cluster from the orignal point cloud
      pcl::PointCloud<segmentation::Point>::Ptr cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl::copyPointCloud( *raw_cloud_ptr, cluster_indices[i], *cluster_ptr);

      pcl::PointCloud<segmentation::Point>::Ptr transformed_cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl::copyPointCloud( *transformed_cloud_ptr, cluster_indices[i], *cluster_ptr);
      
      
      // Transform the cloud to the world frame
      pcl::PointCloud<segmentation::Point>::Ptr transformed_cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl_ros::transformPointCloud( *cluster_ptr, *transformed_cluster_ptr, transform);       

      
      // Filter the transformed point cloud 
      this->filter (transformed_cluster_ptr);
      if( transformed_cluster_ptr->points.empty() )
	continue;
      
      
      // Reverser transform to get the contour right
      pcl_ros::transformPointCloud( *transformed_cluster_ptr, *cluster_ptr, transform.inverse());       
      */
      
      // Post-process the cluster
      try {

	// Create the cluster image 
	cv::Mat cluster_img( img.size(), CV_8U, cv::Scalar(0));
	BOOST_FOREACH  ( const segmentation::Point pt, cluster_ptr->points ) {
	  cv::Point3d pt_cv( pt.x, pt.y, pt.z);
	  cv::Point2f p = cam_model.project3dToPixel(pt_cv);
	  circle( cluster_img, p, 1, cv::Scalar(255), -1, 8, 0 );
	}

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
	  
	  // Draw the contour (for display)
	  cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
	  //cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
	  //cv::Scalar color = cv::Scalar::all(64); // Dark gray
	  cv::drawContours( img, contours, -1, color, 1);
      	  
	  /*
	  // Transform the cloud to the world frame
	  pcl::PointCloud<segmentation::Point>::Ptr transformed_cluster_ptr (new pcl::PointCloud<segmentation::Point>);
	  pcl_ros::transformPointCloud( *cluster_ptr, *transformed_cluster_ptr, transform);
	  
	  // Filter the transformed point cloud 
	  this->filter (transformed_cluster_ptr);
	  if( transformed_cluster_ptr->points.empty() )
	    continue;
	  */

	  /*
	  // Create a transformed point cluster 
	  pcl::PointCloud<segmentation::Point>::Ptr transformed_cluster_ptr (new pcl::PointCloud<segmentation::Point>);
	  pcl::copyPointCloud( *transformed_cloud_ptr, cluster_indices[i], *transformed_cluster_ptr);
	  //pcl::PointCloud<segmentation::Point> transformed_cluster;
	  */

	  // 1. Extract the position
	  geometry_msgs::PoseStamped pose;
	  pose.header.stamp = cloud_msg->header.stamp;
	  //obj.position.data.header.stamp = cloud_msg->header.stamp;
	  //segmentation::getPosition( cluster_ptr, obj.position.data.pose );
	  //segmentation::getPosition( transformed_cluster.makeShared(), obj.position.data.pose );
	  
	  segmentation::getPosition( transformed_cluster_ptr, pose.pose);
	  //segmentation::getPosition( transformed_cluster.makeShared(), pose.pose );
	  obj.position.data = pose;

	  /*
	  // --[ Intially filtering of the whole point cloud instead ]--
	  if( !( obj.position.data.pose.position.x > 0.2  && obj.position.data.pose.position.y > 0.0 && obj.position.data.pose.position.y < 0.62 && obj.position.data.pose.position.z < 0.5 ) )
	    continue;
	  */

	  // Add the position to the movement array
	  movements.movements.push_back(pose);

	  // Add segmented object to display image
	  img.copyTo( result, cluster_img);
	  

	  //std::cout << "Position: [" << obj.position.data.pose.position.x << ", " << obj.position.data.pose.position.y << ", " << obj.position.data.pose.position.z << "]" << std::endl;	 

	  // 2. Extract the shape
	  segmentation::getShape( transformed_cluster_ptr, obj.shape.data );
	  //segmentation::getShape( transformed_cluster.makeShared(), obj.shape.data );

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
	    std::cout << "Position [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "]" << std::endl;
	  */
	  
	  // Add the (un-transformed) cluster to the array message
	  geometry_msgs::Pose center;
	  segmentation::getPosition( cluster_ptr, center );
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
      move_pub_.publish(movements);      
    }
    
  }
}

void ObjectSegmentation::filter( pcl::PointCloud<segmentation::Point>::Ptr &cloud_ptr ) {


  // Filter the transformed point cloud
  if( cloud_ptr->isOrganized ()) {
    segmentation::passThroughFilter( cloud_ptr, cloud_ptr, "x", this->min_x_, this->max_x_); 
    segmentation::passThroughFilter( cloud_ptr, cloud_ptr, "y", this->min_y_, this->max_y_); 
    segmentation::passThroughFilter( cloud_ptr, cloud_ptr, "z", this->min_z_, this->max_z_); 
    /*
    for( auto &p: cloud_ptr->points) {
      if( !( p.x > this->min_x_ && p.x < this->max_x_ ) ||
	  !( p.y > this->min_y_ && p.y < this->max_y_ ) ||
	  !( p.z > this->min_z_ && p.z < this->max_z_ ) ) {
	p.x = std::numeric_limits<double>::quiet_NaN(); //infinity();
	p.y = std::numeric_limits<double>::quiet_NaN();
	p.z = std::numeric_limits<double>::quiet_NaN();
	p.b = p.g = p.r = 0;
      }
    }
    */
  }
  else {
    pcl::PointCloud<segmentation::Point>::Ptr result_ptr (new pcl::PointCloud<segmentation::Point>);
    for( auto &p: cloud_ptr->points) {
      if( ( p.x > this->min_x_ && p.x < this->max_x_ ) &&
	  ( p.y > this->min_y_ && p.y < this->max_y_ ) &&
	  ( p.z > this->min_z_ && p.z < this->max_z_ ) ) {
	result_ptr->points.push_back (p);
      }
    }
    cloud_ptr.swap (result_ptr);
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
