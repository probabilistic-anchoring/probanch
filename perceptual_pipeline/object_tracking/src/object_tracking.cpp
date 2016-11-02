
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/search/pcl_search.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <object_tracking/object_tracking.hpp>

// Macros
#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
  {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
    {                                           \
      std::cout << "Average framerate ("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
  }
// ---------------------------------

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

// ------------------------------
// Tracker struct contructor
// ------------------------------
ObjectTracking::Tracker::Tracker( const pcl::PointCloud<Point>::Ptr &cluster_ptr, 
				  Eigen::Vector4f &center,
				  bool use_fixed ) : center_(center), activity_(MIN_ACTIVITY) {
  

  // Set initial parameters
  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;
    
  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
  
  if (use_fixed) {
    boost::shared_ptr<ParticleFilterOMPTracker<Point, Particle> > tracker
      (new ParticleFilterOMPTracker<Point, Particle> (8));
    tracker_ = tracker;
  }
  else {
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
    tracker_ = tracker;
  }	

  // Set common parameters 
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (600);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseChangeDetector (true);
  tracker_->setUseNormal (false);

  // Set coherences
  ApproxNearestPairPointCloudCoherence<Point>::Ptr coherence = ApproxNearestPairPointCloudCoherence<Point>::Ptr
    (new ApproxNearestPairPointCloudCoherence<Point> ());
    
  boost::shared_ptr<DistanceCoherence<Point> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<Point> > (new DistanceCoherence<Point> ());
  coherence->addPointCoherence (distance_coherence);
    
  boost::shared_ptr<HSVColorCoherence<Point> > color_coherence
    = boost::shared_ptr<HSVColorCoherence<Point> > (new HSVColorCoherence<Point> ());
  color_coherence->setWeight (0.2);
  coherence->addPointCoherence (color_coherence);

  // Set search method 	
  //boost::shared_ptr<pcl::search::KdTree<Point> > search (new pcl::search::KdTree<Point>);
  //boost::shared_ptr<pcl::search::Octree<Point> > search (new pcl::search::Octree<Point> (0.01));
  boost::shared_ptr<pcl::search::OrganizedNeighbor<Point> > search (new pcl::search::OrganizedNeighbor<Point>);    
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (0.01);
  tracker_->setCloudCoherence (coherence);
  
  // Set reference cloud
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  trans.translation ().matrix () = Eigen::Vector3f ( center[0], center[1], center[2]);
  pcl::PointCloud<Point>::Ptr transformed_ptr (new pcl::PointCloud<Point>) ;	
  pcl::transformPointCloud<Point> ( *cluster_ptr, *transformed_ptr, trans.inverse());
  tracker_->setReferenceCloud (transformed_ptr);
  tracker_->setTrans (trans);
  tracker_->setMinIndices (int (cluster_ptr->size ()) / 2);   
}

// Activity functions
void ObjectTracking::Tracker::setActivity(int activity) {
  activity_ += activity;
  activity_ = (activity_ > MAX_ACTIVITY ? MAX_ACTIVITY :		\
	       (activity_ < MIN_ACTIVITY ? MIN_ACTIVITY : activity_) );
}  
float ObjectTracking::Tracker::getActivity() {
  return (activity_ - MIN_ACTIVITY) / (float)MAX_ACTIVITY;
}  
bool ObjectTracking::Tracker::isActive() {
  return (activity_ > MIN_ACTIVITY ? true : false);
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
  vector<vector<Point2i> > contours;  // <-- Note. OpenCV point!
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

  // Activate particle filters and track objects
  this->activate (pts);

  // Downsample the point cloud
  pcl::PointCloud<Point>::Ptr downsampled_cloud_ptr (new pcl::PointCloud<Point>);
  gridSampleApprox ( raw_cloud_ptr, downsampled_cloud_ptr);
  pcl::PointCloud<Point>::Ptr filtered_cloud_ptr (new pcl::PointCloud<Point>);
  radiusSearch ( downsampled_cloud_ptr, filtered_cloud_ptr);

  // Track the particle filters
  this->process (filtered_cloud_ptr);
  
  // Draw reult (with use of camera information)
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(camera_info_msg);
  this->drawParticles (result, cam_model);
    
  // OpenCV window for display
  addWeighted( result, 0.6, gray, 0.4, 0.0, result);
  cv::imshow( "Object tracking...", result );
  char key = cv::waitKey(1); 

  // Transform the cloud to the world frame
  //pcl::PointCloud<Point>::Ptr transformed_cloud_ptr (new pcl::PointCloud<Point>);
  //pcl_ros::transformPointCloud( *raw_cloud_ptr, *transformed_cloud_ptr, transform);

}
  

// ---------------------------------------
// Clusters callback function (ROS)
// -----------------------------------------
void ObjectTracking::clustersCallback (const anchor_msgs::ClusterArray::ConstPtr &msg)  {
  boost::mutex::scoped_lock lock (mtx_);
  static bool got_it = false;

  if( !got_it ) {
    // Receive and store all segmented clusters
    for ( auto i = 0; i < msg->clusters.size(); i++) {
    
      pcl::PointCloud<Point>::Ptr raw_cluster_ptr (new pcl::PointCloud<Point>) ;   
      pcl::fromROSMsg ( msg->clusters[i], *raw_cluster_ptr);
      pcl::PointCloud<Point>::Ptr downsampled_cluster_ptr (new pcl::PointCloud<Point>) ;
      gridSampleApprox ( raw_cluster_ptr, downsampled_cluster_ptr);
    
      Eigen::Vector4f center;
      center[0] = msg->centers[i].position.x;
      center[1] = msg->centers[i].position.y;
      center[2] = msg->centers[i].position.z;
   
      trackers_.push_back ( Tracker( downsampled_cluster_ptr, center) );
    }
    got_it = true;
  }
}


// --------------------------------
// Tracking helper functions
// ----------------------------------------------
void ObjectTracking::activate (const pcl::PointCloud<Point>::Ptr &pts) {

  // Init a list of object trackerers
  for ( auto i = 0; i < trackers_.size(); i++) {
    BOOST_FOREACH  ( const Point pt, pts->points ) {
      float d = distance( pt, trackers_[i].center_);
      trackers_[i].setActivity(d * MAX_ACTIVITY);
      /*
      if( distance( pt, trackers_[i].center_) > 0.9 ) {	
	trackers_[i].setActivity(MAX_ACTIVITY);
      }
      */
    }
    trackers_[i].setActivity(DECREASE_ACTIVITY);
  }
}

// The processing tracking function
void ObjectTracking::process (const pcl::PointCloud<Point>::Ptr &cloud_ptr) {
  boost::mutex::scoped_lock lock (mtx_);
  
  FPS_CALC_BEGIN;
  for (size_t i = 0; i < trackers_.size(); i++) {
    if (trackers_[i].isActive()) {
      trackers_[i].tracker_->setInputCloud (cloud_ptr);
      trackers_[i].tracker_->compute ();
    }
  }
  FPS_CALC_END("tracking");
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
    if (trackers_[i].isActive()) {
      ParticleFilter::PointCloudStatePtr particles = trackers_[i].tracker_->getParticles ();
      if (particles) {
	
	// Draw (tracked) reference cloud
	Scalar blue( trackers_[i].getActivity() * 255, 0, 0);
	Particle result = trackers_[i].tracker_->getResult ();
	Eigen::Affine3f transformation = trackers_[i].tracker_->toEigenMatrix (result);
	pcl::PointCloud<Point>::Ptr result_cloud (new pcl::PointCloud<Point>);
	pcl::transformPointCloud<Point> (*(trackers_[i].tracker_->getReferenceCloud ()), *result_cloud, transformation);
	BOOST_FOREACH  ( const Point pt, result_cloud->points ) {
	  cv::Point3d pt_cv( pt.x, pt.y, pt.z);
	  cv::Point2f p = model.project3dToPixel(pt_cv);
	  circle( img, p, 1, blue, -1, 8, 0 );
	}

	// Draw particles
	Scalar red( 0, 0, trackers_[i].getActivity() * 255);
	BOOST_FOREACH  ( const Particle pt, particles->points ) {
	  cv::Point3d pt_cv( pt.x, pt.y, pt.z);
	  cv::Point2f p = model.project3dToPixel(pt_cv);
	  circle( img, p, 1, red, -1, 8, 0 );
	}
      }
    }
  }
}

// Downsample a point cloud (for improved performance)
void ObjectTracking::gridSampleApprox ( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
					pcl::PointCloud<Point>::Ptr &result_ptr, 
					double leaf_size ) {
  FPS_CALC_BEGIN;
  //pcl::VoxelGrid<PointType> grid;
  pcl::ApproximateVoxelGrid<Point> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud_ptr);
  grid.filter (*result_ptr);
  FPS_CALC_END("gridSample");
}

// Reduce the point cloud to the points closest to active objects 
void ObjectTracking::radiusSearch ( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
				    pcl::PointCloud<Point>::Ptr &result_ptr, 
				    double radius ) {

  // Extract centre ponts of active/inactive objects
  pcl::PointCloud<Point>::Ptr pos_pts (new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr neg_pts (new pcl::PointCloud<Point>);  
  for (size_t i = 0; i < trackers_.size(); i++) {
    Point p;
    p.x = trackers_[i].center_[0]; 
    p.y = trackers_[i].center_[1]; 
    p.z = trackers_[i].center_[2];
    if (trackers_[i].isActive()) {
      pos_pts->push_back (p);
    }
    else {
      neg_pts->push_back (p);
    }   
  }

  // Create a KD-search tree
  pcl::KdTreeFLANN<Point> kdtree;
  kdtree.setInputCloud (cloud_ptr);

  // Search points within a given radius
  std::map<int, float> ptsResult;
  std::map<int, float>::iterator ite;
  BOOST_FOREACH  ( const Point pt, pos_pts->points ) {
    std::vector<int> posPtsIdx;
    std::vector<float> posPtsDistance;
    if( kdtree.radiusSearch ( pt, radius, posPtsIdx, posPtsDistance) > 0 ) {
      for (size_t i = 0; i < posPtsIdx.size(); i++) {
	ite = ptsResult.find (posPtsIdx[i]);
	if ( ite != ptsResult.end() ) {
	  if ( posPtsDistance[i] < ite->second ) {
	    ite->second = posPtsDistance[i];
	  }
	}
	else {
	  ptsResult[ posPtsIdx[i] ] = posPtsDistance[i];
	}
      }
    }
  }
  BOOST_FOREACH  ( const Point pt, neg_pts->points ) {  
    std::vector<int> negPtsIdx;
    std::vector<float> negPtsDistance;
    if( kdtree.radiusSearch ( pt, radius, negPtsIdx, negPtsDistance) > 0 ) {
      for (size_t i = 0; i < negPtsIdx.size(); i++) {
	ite = ptsResult.find (negPtsIdx[i]);
	if ( ite != ptsResult.end() ) {
	  if ( ite->second > negPtsDistance[i] ) {
	    ptsResult.erase (ite);
	  }
	}
      }
    }
  }

  // Create the result cloud
  for ( ite = ptsResult.begin(); ite != ptsResult.end(); ++ite) {
    result_ptr->push_back (cloud_ptr->points[ ite->first ]); 
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

