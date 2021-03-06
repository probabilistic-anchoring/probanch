
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
#include <geometry_msgs/TransformStamped.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/String.h>
#include <anchor_msgs/ObjectArray.h>
#include <anchor_msgs/ClusterArray.h>
#include <anchor_msgs/MovementArray.h>

#include <hand_tracking/TrackingService.h>

#include <object_segmentation/object_segmentation.hpp>

#include <pcl_ros/point_cloud.h>


// ------------------------
// Public functions
// -------------------------
ObjectSegmentation::ObjectSegmentation(ros::NodeHandle nh, bool useApprox) 
  : nh_(nh)
  , it_(nh)
  , priv_nh_("~")
  , tf2_listener_(buffer_)
  , useApprox_(useApprox)
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

  // Hand tracking 
  _tracking_client = nh_.serviceClient<hand_tracking::TrackingService>("/hand_tracking");
  
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

  // Read the base frame
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

  // Read toggle paramter for displayig the result (OpenCV window view)
  this->priv_nh_.param<bool>( "display_window", display_window_, false);
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
  //delete tf_listener_;
}

void ObjectSegmentation::spin() {
  ros::Rate rate(100);
  while(ros::ok()) {

    // OpenCV window for display
    if( this->display_window_ ) {
       if( !this->result_img_.empty() ) {
	 cv::imshow( "Segmented clusters...", this->result_img_ );
       }

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
  
  // Get the transformation (as geoemtry message)
  geometry_msgs::TransformStamped tf_forward, tf_inverse;
  try{
    tf_forward = this->buffer_.lookupTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp);
    tf_inverse = this->buffer_.lookupTransform( cloud_msg->header.frame_id, base_frame_, cloud_msg->header.stamp);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("[ObjectSegmentation::callback] %s" , ex.what());
    return;
  }

  // Get the transformation (as tf/tf2)
  //tf2::Stamped<tf2::Transform> transform2;
  //tf2::fromMsg(tf_forward, transform2);
  //tf::Vector3d 
  tf::Transform transform = tf::Transform(transform2.getOrigin(), transform2.getRotation());
  
  
  //tf::Transform transfrom = tf::Transform(tf_forward.transfrom);
    
  // Read the cloud
  pcl::PointCloud<segmentation::Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl::fromROSMsg (*cloud_msg, *raw_cloud_ptr);
  
  // Transform the cloud to the world frame
  pcl::PointCloud<segmentation::Point>::Ptr transformed_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  //pcl_ros::transformPointCloud( *raw_cloud_ptr, *transformed_cloud_ptr, transform);
  pcl_ros::transformPointCloud(tf2::transformToEigen(tf_forward.transform).matrix(), *raw_cloud_ptr, *transformed_cloud_ptr);

  // Filter the transformed point cloud 
  this->filter (transformed_cloud_ptr);
  // ----------------------
  pcl::PointCloud<segmentation::Point>::Ptr original_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  //pcl_ros::transformPointCloud( *transformed_cloud_ptr, *original_cloud_ptr, transform.inverse());
  pcl_ros::transformPointCloud(tf2::transformToEigen(tf_inverse.transform).matrix(), *raw_cloud_ptr, *transformed_cloud_ptr);
  
  // Read the RGB image
  cv::Mat img;
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

  // TEST
  // --------------------------------------
  // Downsample the orignal point cloud 
  int scale = 2;
  pcl::PointCloud<segmentation::Point>::Ptr downsampled_cloud_ptr( new pcl::PointCloud<segmentation::Point> );					   
  downsampled_cloud_ptr->width = original_cloud_ptr->width / scale;
  downsampled_cloud_ptr->height = original_cloud_ptr->height / scale;
  downsampled_cloud_ptr->resize( downsampled_cloud_ptr->width * downsampled_cloud_ptr->height );
  for( uint i = 0, k = 0; i < original_cloud_ptr->width; i += scale, k++ ) {
    for( uint j = 0, l = 0; j < original_cloud_ptr->height; j += scale, l++ ) {
      downsampled_cloud_ptr->at(k,l) = original_cloud_ptr->at(i,j);
    }
  }
  // -----------
  
  // Convert to grayscale and back (for display purposes)
  if( display_image_ || display_window_) {
    cv::cvtColor( img, this->result_img_, CV_BGR2GRAY); 
    cv::cvtColor( this->result_img_, this->result_img_, CV_GRAY2BGR);
    this->result_img_.convertTo( this->result_img_, -1, 1.0, 50); 
  }
  
  // Camera information
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info_msg);

  // Make a remote call to get the 'hand' (or glove)
  //double t = this->timerStart();
  std::map< int, pcl::PointIndices > hand_indices;
  std::vector< std::vector<cv::Point> > hand_contours; 
 
  hand_tracking::TrackingService srv;
  srv.request.image = *image_msg;
  if( this->_tracking_client.call(srv)) {

    // Get the hand mask contour
    for( int i = 0; i < srv.response.contours.size(); i++) {
      std::vector<cv::Point> contour;
      for( uint j = 0; j < srv.response.contours[i].contour.size(); j++) {
	cv::Point p( srv.response.contours[i].contour[j].x, srv.response.contours[i].contour[j].y );
	contour.push_back(p);
      }
      hand_contours.push_back(contour);
    }

    // Filter the indices of all 3D points within the hand contour
    int key = 0;
    for ( int i = 0; i < hand_contours.size(); i++) {
      cv::Mat mask( img.size(), CV_8U, cv::Scalar(0));
      cv::drawContours( mask, hand_contours, i, cv::Scalar(255), -1);

      uint idx = 0;
      pcl::PointIndices point_idx;
      BOOST_FOREACH  ( const segmentation::Point pt, downsampled_cloud_ptr->points ) {
	tf2::Vector3 trans_pt( pt.x, pt.y, pt.z);
	//tf2::doTransform( trans_pt, trans_pt, tf_forward);
	trans_pt = transform * trans_pt;  
	if( ( trans_pt.x() > this->min_x_ && trans_pt.x() < this->max_x_ ) &&
	    ( trans_pt.y() > this->min_y_ && trans_pt.y() < this->max_y_ ) &&
	    ( trans_pt.z() > this->min_z_ && trans_pt.z() < this->max_z_ ) ) {
	  cv::Point3d pt_3d( pt.x, pt.y, pt.z);
	  cv::Point pt_2d = cam_model.project3dToPixel(pt_3d);
	  if ( pt_2d.y >= 0 && pt_2d.y < mask.rows && pt_2d.x >= 0 && pt_2d.x < mask.cols ) { 
	    if( mask.at<uchar>( pt_2d.y, pt_2d.x) != 0 ) { 
	      point_idx.indices.push_back(idx);
	    }
	  }
	}
	idx++;	
      }
      //ROS_WARN("[TEST] Points: %d", (int)point_idx.indices.size());      
      if( point_idx.indices.size() >= this->seg_.getClusterMinSize() )
	hand_indices.insert( std::pair< int, pcl::PointIndices>( key, point_idx) );
      key++;
    }
  }
  //this->timerEnd( t, "Hand detection");  

  // Cluster cloud into objects 
  // ----------------------------------------
  std::vector<pcl::PointIndices> cluster_indices;
  //this->seg_.clusterOrganized(transformed_cloud_ptr, cluster_indices);
  //this->seg_.clusterOrganized(raw_cloud_ptr, cluster_indices);

  // TEST - Use downsampled cloud instead
  // -----------------------------------------
  //t = this->timerStart();
  this->seg_.clusterOrganized( downsampled_cloud_ptr, cluster_indices);
  //this->timerEnd( t, "Clustering");

  // Post-process the segmented clusters (filter out the 'hand' points)
  //t = this->timerStart();  
  if( !hand_indices.empty() ) {
    for( uint i = 0; i < hand_indices.size(); i++ ) {
      for ( uint j = 0; j < cluster_indices.size (); j++) {
	for( auto &idx : hand_indices[i].indices) {
	  auto ite = std::find (cluster_indices[j].indices.begin(), cluster_indices[j].indices.end(), idx);
	  if( ite != cluster_indices[j].indices.end() ) {
	    cluster_indices[j].indices.erase(ite);
	  }
	}
	if( cluster_indices[j].indices.size() < this->seg_.getClusterMinSize() )
	  cluster_indices.erase( cluster_indices.begin() + j );
      }
    }

    // Add the 'hand' indecies to the reaming cluster indices
    for( auto &ite : hand_indices ) {
      cluster_indices.insert( cluster_indices.begin() + ite.first,  ite.second );
    }
    //ROS_INFO("Clusters: %d (include a 'hand' cluster)", (int)cluster_indices.size());
  }
  else {
    //ROS_INFO("Clusters: %d", (int)cluster_indices.size());
  }
  //this->timerEnd( t, "Post-processing");

  /*
  ROS_INFO("Clusters: \n");
  for (size_t i = 0; i < cluster_indices.size (); i++) {
    if( hand_indices.find(i) != hand_indices.end() )      
      ROS_INFO("Hand size: %d", (int)cluster_indices[i].indices.size());
    else
      ROS_INFO("Object size: %d", (int)cluster_indices[i].indices.size());
  }
  ROS_INFO("-----------");
  */  
  
  // Process all segmented clusters (including the 'hand' cluster)
  if( !cluster_indices.empty() ) {
  
    // Image & Cloud output
    anchor_msgs::ClusterArray clusters;
    anchor_msgs::ObjectArray objects;
    anchor_msgs::MovementArray movements;
    objects.header = cloud_msg->header;
    objects.image = *image_msg;
    objects.info = *camera_info_msg;
    objects.transform = tf_forward;

    /*
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
    */
      
    // Process the segmented clusters
    int sz = 5;
    cv::Mat kernel = cv::getStructuringElement( cv::BORDER_CONSTANT, cv::Size( sz, sz), cv::Point(-1,-1) );
    for (size_t i = 0; i < cluster_indices.size (); i++) {

      // Get the cluster
      pcl::PointCloud<segmentation::Point>::Ptr cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      //pcl::copyPointCloud( *original_cloud_ptr, cluster_indices[i], *cluster_ptr);

      // TEST - Use downsampled cloud instead
      // -----------------------------------------
      pcl::copyPointCloud( *downsampled_cloud_ptr, cluster_indices[i], *cluster_ptr);
      //std::cout << cluster_ptr->points.size() << std::endl;
      
      if( cluster_ptr->points.empty() )
	continue;

            
      // Post-process the cluster
      try {

	// Create the cluster image 
	cv::Mat cluster_img( img.size(), CV_8U, cv::Scalar(0));
	BOOST_FOREACH  ( const segmentation::Point pt, cluster_ptr->points ) {
	  cv::Point3d pt_cv( pt.x, pt.y, pt.z);
	  cv::Point2f p = cam_model.project3dToPixel(pt_cv);
	  circle( cluster_img, p, 2, cv::Scalar(255), -1, 8, 0 );
	}

	// Apply morphological operations the cluster image 
	cv::dilate( cluster_img, cluster_img, kernel);
	cv::erode( cluster_img, cluster_img, kernel);

	// Find the contours of the cluster
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours( cluster_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	// Get the contour of the convex hull
	std::vector<cv::Point> contour = getLargetsContour( contours );
	
	// Create and add the object output message
	if( contour.size() > 0 ) {

	  anchor_msgs::Object obj;
	  for( size_t j = 0; j < contour.size(); j++) {  
	    anchor_msgs::Point2d p;
	    p.x = contour[j].x;
	    p.y = contour[j].y;
	    obj.visual.border.contour.push_back(p);
	  }


	  // Add segmented object to display image
	  img.copyTo( this->result_img_, cluster_img);
	  
	  // Draw the contour (for display)
	  cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
	  //cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
	  //cv::Scalar color = cv::Scalar::all(64); // Dark gray

	  // Check if we have a hand countour
	  if( hand_indices.find(i) != hand_indices.end() ) {
	    color = cv::Scalar( 0, 233, 0); // Green
	  }
	  cv::drawContours( this->result_img_, contours, -1, color, 1);
	    
	  // Transfrom the the cloud once again
	  //tf2::doTransform( *cluster_ptr, *cluster_ptr, tf_forward);
	  //pcl_ros::transformPointCloud( *cluster_ptr, *cluster_ptr, transform);
	  pcl_ros::transformPointCloud(tf2::transformToEigen(tf_forward.transform).matrix(), *cluster_ptr, *cluster_ptr);
	  
	  // 1. Extract the position
	  geometry_msgs::PoseStamped pose;
	  pose.header.stamp = cloud_msg->header.stamp;
	  segmentation::getOrientedPosition( cluster_ptr, pose.pose);
	  obj.position.data = pose;


	  // Add the position to the movement array
	  movements.movements.push_back(pose);	  

	  //std::cout << "Position: [" << obj.position.data.pose.position.x << ", " << obj.position.data.pose.position.y << ", " << obj.position.data.pose.position.z << "]" << std::endl;	 

	  // 2. Extract the shape
	  segmentation::getSize( cluster_ptr, obj.size.data );

	  // Ground size symbols
	  std::vector<double> data = { obj.size.data.x, obj.size.data.y, obj.size.data.z};
	  std::sort( data.begin(), data.end(), std::greater<double>());
	  if( data.front() <= 0.1 ) { // 0 - 15 [cm] = small
	    obj.size.symbols.push_back("small");
	  }
	  else if( data.front() <= 0.20 ) { // 16 - 30 [cm] = medium
	    obj.size.symbols.push_back("medium");
	  }
	  else {  // > 30 [cm] = large
	    obj.size.symbols.push_back("large");
	  }
	  if( data[0] < data[1] * 1.1 ) {
	    obj.size.symbols.push_back("square");
	  }
	  else {
	    obj.size.symbols.push_back("rectangle");
	    if( data[0] > data[1] * 1.5 ) {
	      obj.size.symbols.push_back("long");
	    }
	    else {
	      obj.size.symbols.push_back("short");
	    }
	  }


	  // TEST  
	  // ---------------------------------------
	  // Attempt to filter out glitchs
	  if ( obj.size.data.z < 0.02 )
	    continue;
	  
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
    //std::cout << "---" << std::endl;
    
    // Publish the "segmented" image
    if( display_image_ ) {
      cv_ptr->image = this->result_img_;
      cv_ptr->encoding = "bgr8";
      display_image_pub_.publish(cv_ptr->toImageMsg());
    }
    
    // Publish the object array
    if( !objects.objects.empty() || !clusters.clusters.empty() ) {
      // ROS_INFO("Publishing: %d objects.", (int)objects.objects.size());
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

std::vector<cv::Point> ObjectSegmentation::getLargetsContour( std::vector<std::vector<cv::Point> > contours ) {
  std::vector<cv::Point> result = contours.front();
  for ( size_t i = 1; i< contours.size(); i++)
    if( contours[i].size() > result.size() )
      result = contours[i];
  return result;
}

std::vector<cv::Point> ObjectSegmentation::contoursConvexHull( std::vector<std::vector<cv::Point> > contours ) {
  std::vector<cv::Point> result;
  std::vector<cv::Point> pts;
  for ( size_t i = 0; i< contours.size(); i++)
    for ( size_t j = 0; j< contours[i].size(); j++)
      pts.push_back(contours[i][j]);
  cv::convexHull( pts, result );
  return result;
}

// Detect a 'hand'/'glove' object based on color segmentation
std::vector<cv::Point> ObjectSegmentation::handDetection( cv::Mat &img ) {
  cv::Mat hsv_img;
  int sz = 5;
  
  // Convert to HSV color space
  cv::cvtColor( img, hsv_img, cv::COLOR_BGR2HSV);

  // Pre-process the image by blur
  cv::blur( hsv_img, hsv_img, cv::Size( sz, sz));

  // Extrace binary mask
  cv::Mat mask;   
  cv::inRange( hsv_img, cv::Scalar( 31, 68, 141), cv::Scalar( 47, 255, 255), mask);

  // Post-process the threshold image  
  cv::Mat kernel = cv::getStructuringElement( cv::BORDER_CONSTANT, cv::Size( sz, sz), cv::Point(-1,-1) );
  cv::dilate( mask, mask, kernel);
  cv::erode( mask, mask, kernel);

  // Find the contours 
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours( mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

  // Filter the contoruse based on size
  std::vector<std::vector<cv::Point> > result;
  for ( int i = 0; i < contours.size(); i++) {
    if ( cv::contourArea(contours[i]) > 500 ) {
      result.push_back(contours[i]);
    }
  }
  return result[0];
}

// Timer functions
double ObjectSegmentation::timerStart() {
  return (double)cv::getTickCount();
}
void ObjectSegmentation::timerEnd( double t, std::string msg) {
  t = ((double)cv::getTickCount() - t) / (double)cv::getTickFrequency();
  ROS_INFO("[Timer] %s: %.4f (s)", msg.c_str(), t);
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
