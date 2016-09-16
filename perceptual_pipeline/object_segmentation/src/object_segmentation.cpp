
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

/*
#include <anchoring/ObjectArray.h>
#include <anchoring/CloudArray.h>
#include <anchoring/MovementArray.h>
*/

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

  ROS_WARN("Set up the ROS subscribers... ");

  // Subscribers / publishers
  //image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
  //image_sub_ = new image_transport::SubscriberFilter( it_, topicColor, "image", hints);1

  image_sub_ = new image_transport::SubscriberFilter( it_, "image", queueSize_);
  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>( nh_, "camera_info", queueSize_);
  cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( nh_, "cloud", queueSize_);
  seg_image_pub_ = it_.advertise("/display/segmented_image", 1);
  

  ROS_WARN("Set sync ploicies... ");

  // Set up sync policies
  if(useApprox) {
    syncApproximate_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
    syncApproximate_->registerCallback( boost::bind( &ObjectSegmentation::callback, this, _1, _2, _3));
  }
  else {
    syncExact_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize_), *image_sub_, *camera_info_sub_, *cloud_sub_);
    syncExact_->registerCallback( boost::bind( &ObjectSegmentation::callback, this, _1, _2, _3));
  }
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
}

void ObjectSegmentation::spin() {
  while (ros::ok()) {
    ros::spin();
  }    
}


// --------------------------
// Callback function (ROS)
// --------------------------
void ObjectSegmentation::callback( const sensor_msgs::Image::ConstPtr image, 
				  const sensor_msgs::CameraInfo::ConstPtr camera_info, 
				  const sensor_msgs::PointCloud2::ConstPtr cloud) {

  // Read the cloud
  pcl::PointCloud<segmentation::Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<segmentation::Point>);
  pcl::fromROSMsg (*cloud, *raw_cloud_ptr);
  std::cout << "Cloud size: " << raw_cloud_ptr->width << " x " << raw_cloud_ptr->height << std::endl;

  // Cluster cloud into objects 
  // ----------------------------------------
  segmentation::Segmentation seg(raw_cloud_ptr);
  std::vector<pcl::PointIndices> cluster_indices;
  seg.cluster_organized(cluster_indices, 4);
  if( !cluster_indices.empty() ) {
 
    ROS_WARN("Clusters: %d", (int)cluster_indices.size());

    cv::Mat img;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( image, image->encoding);
      cv_ptr->image.copyTo(img);
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[object_recognition] Failed to convert image.");
      return;
    }

    // Saftey check
    if( img.empty() ) {
      return;
    }
    
    // Camera information
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info);

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
	
	// Add the contour to the output message
	if( contours.size() > 0 ) {
	  /*
	  anchoring::Contour new_contour;
	  for( size_t j = 0; j < contours[0].size(); j++) {  
	    anchoring::Point2d p;
	    p.x = contours[0][j].x;
	    p.y = contours[0][j].y;
	    new_contour.contour.push_back(p);
	  }
	  new_objects.contours.push_back(new_contour);	
	  */

	  // Draw the contour (for display)
	  cv::drawContours( img, contours, -1, cv::Scalar( 0, 0, 255), 2);
      
	  /*
	  // Transform the cloud to the world frame
	  pcl::PointCloud<segmentation::Point> transformed_cloud;
	  pcl_ros::transformPointCloud( *cluster_ptr, transformed_cloud, transform);
	  
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
  }
}


// ----------------------
// Main function
// ------------------
int main(int argc, char **argv) {
  ROS_WARN("Start up the node... ");
  
  ros::init(argc, argv, "object_recognition_node");
  ros::NodeHandle nh;

  // Saftey check
  if(!ros::ok()) {
    return 0;
  }
  ROS_WARN("Create the object... ");

  ObjectSegmentation node(nh);
  node.spin();
  
  ros::shutdown();
  return 0;
}
