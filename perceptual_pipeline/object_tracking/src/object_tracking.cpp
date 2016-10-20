
#include <object_tracking/object_tracking.hpp>

using namespace std;
using namespace pcl::tracking;

// ------------------------
// Public functions
// -------------------------
ObjectTracking::ObjectTracking (ros::NodeHandle nh) 
  : nh_(nh)
  , priv_nh_("~") 
{

  // Subscribers / publishers
  
  // Create transformation listener
  tf_listener_ = new tf::TransformListener();
  priv_nh_.param( "base_frame", base_frame_, std::string("base_link"));
}

void ObjectTracking::spin() {
  while (ros::ok()) {
    ros::spin();
  }    
}

// ---------------------------------------
// Clusters callback function (ROS)
// -----------------------------------------
void ObjectTracking::clustersCallback( const anchor_msgs::ClusterArray::ConstPtr &msg)  {
  boost::mutex::scoped_lock lock (mtx_);

  // Receive and store all segmented clusters
  references_.clear();
  for ( auto i = 0; i < msg->clusters.size(); i++) {
    pcl::PointCloud<Point>::Ptr cluster (new pcl::PointCloud<Point>) ;   
    pcl::fromROSMsg ( msg->clusters[i], *cluster);
    references_.push_back (cluster);
  }
}

// --------------------------------
// Tracking functions
// ----------------------------------------------
void ObjectTracking::init(const pcl::PointCloud<Point>::Ptr &pts) {
  trackers_.clear();
    
  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;
    
  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  // Init a list of object trackerers
  for ( auto i = 0; i < references_.size(); i++) {
    
    Eigen::Vector4f cent;
    pcl::compute3DCentroid<Point> ( *references_[i], cent);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    trans.translation ().matrix () = Eigen::Vector3f (cent[0], cent[1], cent[2]);
    pcl::transformPointCloud<Point> ( *references_[i], *references_[i], trans.inverse());

    //for ( auto j = 0; j < pts.size(); j++ ) {
    BOOST_FOREACH  ( const Point pt, pts->points ) {
      if( distance( pt, cent) > 0.9 ) {

	boost::shared_ptr<ParticleFilterOMPTracker<Point, Particle> > tracker (new ParticleFilterOMPTracker<Point, Particle> (8));
      
	//tracker->setTrans (Eigen::Affine3f::Identity ());
	tracker->setTrans (trans);
	tracker->setStepNoiseCovariance (default_step_covariance);
	tracker->setInitialNoiseCovariance (initial_noise_covariance);
	tracker->setInitialNoiseMean (default_initial_mean);
	tracker->setIterationNum (1);
	tracker->setParticleNum (250);
	tracker->setResampleLikelihoodThr(0.00);
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
	boost::shared_ptr<pcl::search::OrganizedNeighbor<Point> > search (new pcl::search::OrganizedNeighbor<Point>);    

	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.01);
	tracker->setCloudCoherence (coherence);

	trackers_.push_back(tracker);

	break;
      }
    }
  }
}

// Distance function for compareing a point against a cluster center
float ObjectTracking::distance( const Point &pt, const Eigen::Vector4f &center) {
  float dist  = sqrt( (pt.x - center[0]) * (pt.x - center[0]) + 
		      (pt.y - center[1]) * (pt.y - center[1]) +  
		      (pt.z - center[2]) * (pt.z - center[2]) );
  return 1.0 / exp(dist);
}

// Draw the resulting particles on the resulting image
void ObjectTracking::drawParticles () {

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

