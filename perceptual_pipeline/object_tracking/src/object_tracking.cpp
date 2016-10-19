
#include <object_tracking/object_tracking.hpp>

// ------------------------
// Public functions
// -------------------------
ObjectTracking::ObjectTracking (ros::NodeHandle nh) 
  : nh_(nh)
  , useApprox_(useApprox)
  , it_(nh)
  , priv_nh_("~")
  , queueSize_(5) {

}

void ObjectTracking::spin() {
  while (ros::ok()) {
    ros::spin();
  }    
}


// --------------------------------
// Tracking functions
// ----------------------------------------------
void ObjectTracking::init(size_t clusters_size) {
  trackers_.clear();
    
  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;
    
  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  // Init a list of tracker objects
  for(size_t i = 0; i < clusters_size; i++) {
    boost::shared_ptr<ParticleFilterOMPTracker<point, Particle> > tracker (new ParticleFilterOMPTracker<Point, Particle> (8));
      
    tracker->setTrans (Eigen::Affine3f::Identity ());
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

