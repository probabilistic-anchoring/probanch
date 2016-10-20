
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <object_tracking/object_tracking.hpp>

using namespace std;
using namespace cv;
using namespace pcl::tracking;

// ------------------------
// Public functions
// -------------------------

// Constructor
ObjectTracking::ObjectTracking (ros::NodeHandle nh, bool useApprox) 
  : nh_(nh)
  , it_(nh)
  , priv_nh_("~")
  , useApprox_(useApprox)
  , queueSize_(5)
{

  // Subscribers / publishers
  rgb_sub_ = new image_transport::SubscriberFilter( it_, "rgb_image", queueSize_);
  depth_sub_ = new image_transport::SubscriberFilter( it_, "depth_image", queueSize_);
  camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>( nh_, "camera_info", queueSize_);
  cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>( nh_, "cloud", queueSize_);

  clusters_sub_ = nh_.subscribe("/objects/clusters", queueSize_, &ObjectTracking::clustersCallback, this);

  // Set sync policies
  if(useApprox) {
    syncApproximate_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize_), *rgb_sub_, *depth_sub_, *camera_info_sub_, *cloud_sub_);
    syncApproximate_->registerCallback( boost::bind( &ObjectTracking::trackCallback, this, _1, _2, _3, _4));
  }
  else {
    syncExact_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize_), *rgb_sub_, *depth_sub_, *camera_info_sub_, *cloud_sub_);
    syncExact_->registerCallback( boost::bind( &ObjectTracking::trackCallback, this, _1, _2, _3, _4));
  }
  
  // Create transformation listener
  tf_listener_ = new tf::TransformListener();
  priv_nh_.param( "base_frame", base_frame_, std::string("base_link"));


  // Create background subtractors
  depth_subtractor_ = new cv::BackgroundSubtractorMOG2( 500, 16.0f, false);
  rgb_subtractor_ = new cv::BackgroundSubtractorMOG2( 500, 16.0f, true);
}

// Destructor
ObjectTracking::~ObjectTracking () {
  depth_subtractor_.release();
  rgb_subtractor_.release();
    
  if(useApprox_) {
    delete syncApproximate_;
  }
  else {
    delete syncExact_;
  }
  delete rgb_sub_;
  delete depth_sub_;
  delete camera_info_sub_;
  delete cloud_sub_;
}

// ROS access function
void ObjectTracking::spin() {
  while (ros::ok()) {
    ros::spin();
  }    
}

// ------------------------------------------
// Background subtraction function
// ------------------------------------------
void ObjectTracking::trackCallback( const sensor_msgs::Image::ConstPtr &rgb_msg, 
				    const sensor_msgs::Image::ConstPtr &depth_msg, 
				    const sensor_msgs::CameraInfo::ConstPtr &camera_info_msg, 
				    const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {

  // Get the transformation
  tf::StampedTransform transform;
  try{
    tf_listener_->waitForTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.5) ); 
    tf_listener_->lookupTransform( base_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, transform );
  }
  catch( tf::TransformException ex) {
    ROS_WARN("[ObjectTracking::trackCallback] %s" , ex.what());
    return;
  }


  // Read images
  Mat rgb_mask, depth_mask, rgb_frame, depth_frame;
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
    cv_ptr->image.copyTo(rgb_frame);
    depth_frame = cv_bridge::toCvShare( depth_msg, depth_msg->encoding)->image;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("[ObjectTracking::trackCallback] Could not convert image with encoding '%s'.", depth_msg->encoding.c_str());
    return;
  }

  // Read the cloud
  pcl::PointCloud<Point>::Ptr raw_cloud_ptr (new pcl::PointCloud<Point>);
  pcl::fromROSMsg (*cloud_msg, *raw_cloud_ptr);

  // Subtract the background
  Mat frame_lab, gray;
  cvtColor( rgb_frame, frame_lab, CV_BGR2Lab);
  cvtColor( rgb_frame, gray, CV_BGR2GRAY);
  cvtColor( gray, gray, CV_GRAY2BGR);

  double learning_rate = 0.01; 
  rgb_subtractor_->operator()( frame_lab, rgb_mask, learning_rate);
  depth_subtractor_->operator()( depth_frame, depth_mask, learning_rate);
    
  // Saftey check
  if( rgb_mask.empty() ||
      depth_mask.empty() || 
      rgb_mask.size() != depth_mask.size() )
    return;

  // Create the probability image
  Mat mask = min( rgb_mask, depth_mask);
  mask.convertTo( mask, CV_32F);
  if( past_.empty() ) {
    past_ = mask;
  }
  if( prev_.empty() ) {
    prev_ = mask;
  }
  Mat result( past_ * 0.33);
  GaussianBlur( result, result, Size( 9, 9 ), 0, 0 );
  past_ = prev_;
  result = cv::max( result, prev_ * 0.66);
  GaussianBlur( result, result, Size( 7, 7 ), 0, 0 );
  prev_ = mask;
  result = cv::max( result, mask);
  GaussianBlur( result, result, Size( 9, 9 ), 0, 0 );
  result.convertTo( result, CV_8U);
    
  // Saftey check
  if ( result.empty() ) 
    return;
    
  // Find contours
  Mat result_mask;
  cv::threshold( result, result_mask, 200, 255, THRESH_BINARY);
  vector<vector<Point2i> > contours;  // <-- Note. OpenCV point
  vector<Vec4i> hierarchy;
  findContours( result_mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    
  // Process the result
  applyColorMap( result, result, COLORMAP_HOT);    
  pcl::PointCloud<Point>::Ptr pts (new pcl::PointCloud<Point>);
  if( !contours.empty() ) {
    for( int i = 0; i < contours.size(); i++ ) {
      Moments m = moments( contours[i], false );
      Point2f p( m.m10/m.m00 , m.m01/m.m00 );
      circle( result, p, 4, Scalar( 0, 0, 255), -1, 8, 0 );
      if( (int)p.x >= 0 && (int)p.x < raw_cloud_ptr->width &&
	  (int)p.y >= 0 && (int)p.y < raw_cloud_ptr->height ) {
	pts->push_back( raw_cloud_ptr->at( (int)p.x, (int)p.y) );
      }
    }
  }

  // Init particle filters and tarck objects
  //pcl_ros::transformPointCloud( *pts, *pts, transform);
  if( trackers_.empty() )
    this->init (pts);
  //cout << "Active: " << trackers_.size() << " (" << references_.size() << ")" << endl; 
  
  // Transform the cloud to the world frame
  //pcl::PointCloud<Point>::Ptr transformed_cloud_ptr (new pcl::PointCloud<Point>);
  //pcl_ros::transformPointCloud( *raw_cloud_ptr, *transformed_cloud_ptr, transform);
  this->process (raw_cloud_ptr);
    
  // Camera information
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info_msg);
  this->drawParticles (result, cam_model);
  
  // OpenCV window for display
  addWeighted( result, 0.6, gray, 0.4, 0.0, result);
  cv::imshow( "Object tracking...", result );
  char key = cv::waitKey(1); 
}
  

// ---------------------------------------
// Clusters callback function (ROS)
// -----------------------------------------
void ObjectTracking::clustersCallback (const anchor_msgs::ClusterArray::ConstPtr &msg)  {
  boost::mutex::scoped_lock lock (mtx_);
  static bool got_it = false;

  if( !got_it ) {
  // Receive and store all segmented clusters
  references_.clear();
  centers_.clear();
  for ( auto i = 0; i < msg->clusters.size(); i++) {
    pcl::PointCloud<Point>::Ptr raw_cluster_ptr (new pcl::PointCloud<Point>) ;   
    pcl::fromROSMsg ( msg->clusters[i], *raw_cluster_ptr);
    pcl::PointCloud<Point>::Ptr cluster_ptr (new pcl::PointCloud<Point>) ;
    this->removeZeroPoints ( raw_cluster_ptr, cluster_ptr);
    references_.push_back (cluster_ptr);
    
    Eigen::Vector4f center;
    center[0] = msg->centers[i].position.x;
    center[1] = msg->centers[i].position.y;
    center[2] = msg->centers[i].position.z;
    centers_.push_back(center);
  }
  got_it = true;
  }
}


// --------------------------------
// Tracking helper functions
// ----------------------------------------------
void ObjectTracking::init (const pcl::PointCloud<Point>::Ptr &pts) {
  boost::mutex::scoped_lock lock (mtx_);

  trackers_.clear();
    
  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;
    
  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  // Init a list of object trackerers
  for ( auto i = 0; i < references_.size(); i++) {

    //for ( auto j = 0; j < pts.size(); j++ ) {
    BOOST_FOREACH  ( const Point pt, pts->points ) {
      if( distance( pt, centers_[i]) > 0.9 ) {
	/*
	boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<Point, Particle> > tracker 
	  (new KLDAdaptiveParticleFilterOMPTracker<Point, Particle> (8));
	
	Particle bin_size;
	bin_size.x = 0.1f;
	bin_size.y = 0.1f;
	bin_size.z = 0.1f;
	bin_size.roll = 0.1f;
	bin_size.pitch = 0.1f;
	bin_size.yaw = 0.1f;
	tracker->setBinSize (bin_size);

	tracker->setMaximumParticleNum (1000);
	tracker->setDelta (0.99);
	tracker->setEpsilon (0.2);	
	*/

	boost::shared_ptr<ParticleFilterOMPTracker<Point, Particle> > tracker (new ParticleFilterOMPTracker<Point, Particle> (8));
      	tracker->setStepNoiseCovariance (default_step_covariance);
	tracker->setInitialNoiseCovariance (initial_noise_covariance);
	tracker->setInitialNoiseMean (default_initial_mean);
	tracker->setIterationNum (1);
	tracker->setParticleNum (400);
	tracker->setResampleLikelihoodThr(0.00);
	tracker->setUseChangeDetector (true);
	tracker->setUseNormal (false);

	// Setup coherences
	ApproxNearestPairPointCloudCoherence<Point>::Ptr coherence = ApproxNearestPairPointCloudCoherence<Point>::Ptr
	  (new ApproxNearestPairPointCloudCoherence<Point> ());
    
	boost::shared_ptr<DistanceCoherence<Point> > distance_coherence
	  = boost::shared_ptr<DistanceCoherence<Point> > (new DistanceCoherence<Point> ());
	coherence->addPointCoherence (distance_coherence);
    
	boost::shared_ptr<HSVColorCoherence<Point> > color_coherence
	  = boost::shared_ptr<HSVColorCoherence<Point> > (new HSVColorCoherence<Point> ());
	color_coherence->setWeight (0.1);
	coherence->addPointCoherence (color_coherence);

	// Search method used
	
	boost::shared_ptr<pcl::search::Octree<Point> > search (new pcl::search::Octree<Point> (0.01));
	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.01);
	
	/*
	boost::shared_ptr<pcl::search::OrganizedNeighbor<Point> > search (new pcl::search::OrganizedNeighbor<Point>);    
	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.01);
	*/
	tracker->setCloudCoherence (coherence);

	
	// Set reference cloud
	Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
	trans.translation ().matrix () = Eigen::Vector3f ( centers_[i][0], centers_[i][1], centers_[i][2]);
	pcl::PointCloud<Point>::Ptr transformed_ptr (new pcl::PointCloud<Point>) ;	
	pcl::transformPointCloud<Point> ( *references_[i], *transformed_ptr, trans.inverse());
	tracker->setReferenceCloud (transformed_ptr);
	tracker->setTrans (trans);
	tracker->setMinIndices (int (references_[i]->size ()) / 2);   
	
	trackers_.push_back(tracker);

	break;
      }
    }
  }
}

// The processing tracking function
void ObjectTracking::process (const pcl::PointCloud<Point>::Ptr &cloud_ptr) {
  boost::mutex::scoped_lock lock (mtx_);
  
  //FPS_CALC_BEGIN;
  for (size_t i = 0; i < trackers_.size(); i++) {
    trackers_[i]->setInputCloud (cloud_ptr);
    trackers_[i]->compute ();
  }
  //FPS_CALC_END("tracking");
}
  

// Distance function for compareing a point against a cluster center
float ObjectTracking::distance ( const Point &pt, const Eigen::Vector4f &center) {
  float dist  = sqrt( (pt.x - center[0]) * (pt.x - center[0]) + 
		      (pt.y - center[1]) * (pt.y - center[1]) +  
		      (pt.z - center[2]) * (pt.z - center[2]) );
  return 1.0 / exp(dist);
}

// Draw the resulting particles on the resulting image
void ObjectTracking::drawParticles (cv::Mat &img, const image_geometry::PinholeCameraModel &model) {

  for (size_t i = 0; i < trackers_.size(); i++) { 
    ParticleFilter::PointCloudStatePtr particles = trackers_[i]->getParticles ();
    if (particles) {
      pcl::PointCloud<SimplePoint>::Ptr particle_cloud (new pcl::PointCloud<SimplePoint>);
      for (size_t i = 0; i < particles->points.size (); i++) {
	/*
	SimplePoint point;
	point.x = particles->points[i].x;
	point.y = particles->points[i].y;
	point.z = particles->points[i].z;
	particle_cloud->points.push_back (point);
	*/
	cv::Point3d pt_cv( particles->points[i].x, 
			   particles->points[i].y, 
			   particles->points[i].z );
	cv::Point2f p = model.project3dToPixel(pt_cv);
	circle( img, p, 2, Scalar( 255, 0, 0), -1, 8, 0 );
      }
    }
  }
}

void  ObjectTracking::removeZeroPoints ( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
					 pcl::PointCloud<Point>::Ptr &result_ptr ) {
  for ( auto p = cloud_ptr->begin(); p != cloud_ptr->end(); ++p) {
    //Point p = *cloud.points[i];
    if (!( fabs(p->x) < 0.01 &&
	   fabs(p->y) < 0.01 &&
	   fabs(p->z) < 0.01 ) &&
	!pcl_isnan(p->x) &&
	!pcl_isnan(p->y) &&
	!pcl_isnan(p->z) ) {
      result_ptr->push_back(*p);
    }
  }
}


// ----------------------
// Main function
// ------------------
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "object_tracking_node");
  ros::NodeHandle nh;

  // Saftey check
  if(!ros::ok()) {
    return 0;
  }

  ObjectTracking node(nh);
  node.spin();
  
  ros::shutdown();
  return 0;
}

