
#include <algorithm>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include <anchor_msgs/ObjectArray.h>

#include <object_segmentation/object_segmentation.hpp>

// ------------------------
// Public functions
// -------------------------
ObjectSegmentation::ObjectSegmentation(ros::NodeHandle nh, bool useApprox) 
  : nh_(nh)
  , useApprox_(useApprox)
  , it_(nh)
  , priv_nh_("~")
  , queueSize_(5) {

  // Subscribers / publishers
  //image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
  //image_sub_ = new image_transport::SubscriberFilter( it_, topicColor, "image", hints);

  image_sub_ = new image_transport::SubscriberFilter( it_, "image", queueSize_);
  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>( nh_, "camera_info", queueSize_);
  cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( nh_, "cloud", queueSize_);
  seg_image_pub_ = it_.advertise("/display/segmented", 1);

  obj_pub_ = nh_.advertise<anchor_msgs::ObjectArray>("/objects/raw", queueSize_);
  
  // Set up sync policies
  if(useApprox) {
    syncApproximate_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
    syncApproximate_->registerCallback( boost::bind( &ObjectSegmentation::callback, this, _1, _2, _3));
  }
  else {
    syncExact_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
    syncExact_->registerCallback( boost::bind( &ObjectSegmentation::callback, this, _1, _2, _3));
  }

  // Create transformation listener
  tf_listener_ = new tf::TransformListener();
  priv_nh_.param( "base_frame", base_frame_, std::string("base_link"));
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


// --------------------------
// Callback function (ROS)
// --------------------------
void ObjectSegmentation::callback( const sensor_msgs::Image::ConstPtr image_msg, 
				   const sensor_msgs::CameraInfo::ConstPtr camera_info_msg, 
				   const sensor_msgs::PointCloud2::ConstPtr cloud_msg) {
  
  // Get the transformation
  tf::StampedTransform transform;
  try{
    tf_listener_->waitForTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.5) ); 
    tf_listener_->lookupTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, transform );
  }
  catch( tf::TransformException ex) {
    ROS_WARN("[TabletopSegmentor::process] %s" , ex.what());
    return;
  }

  // Read the cloud
  pcl::PointCloud<segmentation::Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl::fromROSMsg (*cloud_msg, *raw_cloud_ptr);
  //std::cout << "Cloud size: " << raw_cloud_ptr->width << " x " << raw_cloud_ptr->height << std::endl;


  // Cluster cloud into objects 
  // ----------------------------------------
  segmentation::Segmentation seg(raw_cloud_ptr);
  std::vector<pcl::PointIndices> cluster_indices;
  seg.cluster_organized(cluster_indices, 4);
  if( !cluster_indices.empty() ) {
 
    //ROS_WARN("Clusters: %d", (int)cluster_indices.size());

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
    
    // Camera information
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info_msg);

    // Image & Cloud output
    anchor_msgs::ObjectArray objects;
    objects.header = cloud_msg->header;
    objects.image = *image_msg;

    // Process the segmented clusters
    for (size_t i = 0; i < cluster_indices.size (); i++) {
      
      // Create the point cluster
      pcl::PointCloud<segmentation::Point>::Ptr cluster_ptr (new pcl::PointCloud<segmentation::Point>);
      pcl::copyPointCloud( *raw_cloud_ptr, cluster_indices[i], *cluster_ptr);
  
      // Pots-process the cluster
      std::vector<pcl::Vertices> indices;
      seg.post_process( cluster_ptr, indices);
      try {

	// Create the cluster image 
	cv::Mat cluster_img( img.size(), CV_8U, cv::Scalar(0));
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
	    obj.border.contour.push_back(p);
	  }
	  
	  // Draw the contour (for display)
	  cv::drawContours( img, contours, -1, cv::Scalar( 0, 0, 255), 2);
      	  
	  // Transform the cloud to the world frame
	  pcl::PointCloud<segmentation::Point> transformed_cloud;
	  pcl_ros::transformPointCloud( *cluster_ptr, transformed_cloud, transform);
	  
	  // 1. Extract the location
	  obj.location.data.header.stamp = cloud_msg->header.stamp;
	  segmentation::getLocation( raw_cloud_ptr, obj.location.data.pose );
	  //if( obj.location.data.pose.position.x < 0.2  || obj.location.data.pose.position.y < 0.0 || obj.location.data.pose.position.y > 0.62 || obj.location.data.pose.position.z > 0.5 )
	  //  continue;
    
	  // 2. Extract the shape
	  segmentation::getShape( raw_cloud_ptr, obj.shape.data );

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
	  /*
	  sensor_msgs::PointCloud2 pc_object;
	  pcl::toROSMsg( transformed_cloud, pc_object );
	  //pcl::concatenatePointCloud( pc_multi, pc_object, pc_multi);
	  new_clouds.clouds.push_back(pc_object);
	  */
	}
      }
      catch( cv::Exception &exc ) {
	ROS_ERROR("[object_recognition] CV processing error: %s", exc.what() );
      }
    }
    
    // Publish the segmented image (with contours)
    cv_ptr->image = img;
    cv_ptr->encoding = "bgr8";
    seg_image_pub_.publish(cv_ptr->toImageMsg());

    
    // Publish the object array
    if( !objects.objects.empty() ) {
      //ROS_INFO("Publishing: %d objects.", (int)objects.objects.size());
      obj_pub_.publish(objects);
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
