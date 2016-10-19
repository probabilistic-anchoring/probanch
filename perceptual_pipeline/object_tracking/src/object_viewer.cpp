
// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>

// PCL specific includes

// PCL feature includes
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

// PCL comparators
#include <pcl/segmentation/comparator.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>


#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>


#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

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
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
    }                                           \
    else                                        \
    {                                           \
      duration += end_time - start_time;        \
    }                                           \
}
// ---------------------------------

using namespace pcl::tracking;

template <typename PointType>
class ObjectViewer
{
public:
  
  // Typedefs
  typedef pcl::PointXYZRGBA RefPointType;
  typedef ParticleXYZRPY ParticleT;
  
  typedef pcl::PointCloud<PointType> Cloud;
  typedef pcl::PointCloud<RefPointType> RefCloud;
  typedef typename RefCloud::Ptr RefCloudPtr;
  typedef typename RefCloud::ConstPtr RefCloudConstPtr;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
  typedef typename ParticleFilter::CoherencePtr CoherencePtr;
  typedef typename pcl::search::KdTree<PointType> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;

  ObjectViewer (ros::NodeHandle nh) 
  : nh_(nh) 
  , viewer_ ("PCL Object Viewer")
  , cloud_ (new Cloud)
  , cloud_ref_ (new Cloud)
  , new_cloud_ (false)
  , have_clusters_ (false)
  , clusters_size_ (0)
  , factor_ (0.02f)
  , size_ (20.0f)
  , planeMinSize_ (10000)
  , clusterMinSize_ (500)
  , angularTh_  (2.0f)
  , distanceTh_ (0.01f)
  {

    // Init the PCL viewer
    viewer_.runOnVisualizationThread ( boost::bind( &ObjectViewer::viz_cb, this, _1), "viz_cb");
    viewer_.registerKeyboardCallback ( &ObjectViewer::keyboard_callback, *this, 0);

    // Create a ROS subscriber for the input point cloud
    sub_ = nh_.subscribe ("cloud", 1, &ObjectViewer::cloud_cb, this);

    // Create comparator objects
    plane_comparator_.reset (new pcl::PlaneCoefficientComparator<PointType, pcl::Normal> ());
    euclidean_comparator_.reset (new pcl::EuclideanPlaneCoefficientComparator<PointType, pcl::Normal> ());
    rgb_comparator_.reset (new pcl::RGBPlaneCoefficientComparator<PointType, pcl::Normal> ());
    edge_aware_comparator_.reset (new pcl::EdgeAwarePlaneComparator<PointType, pcl::Normal> ());
    euclidean_cluster_comparator_.reset (new pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label> ());

    // Set default comparator used for refinement
    mps_.setComparator (edge_aware_comparator_);
  }

  // -------------------------
  // PCL visualization functions
  // ------------------------------------
  
  void
  drawParticles (pcl::visualization::PCLVisualizer& viz)
  {
    char name[1024];
    for (size_t i = 0; i < trackers_.size(); i++) { 
      sprintf (name, "cluster_particles_%d" , int (i));
      ParticleFilter::PointCloudStatePtr particles = trackers_[i]->getParticles ();
      if (particles) {
	
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (size_t i = 0; i < particles->points.size (); i++) {
          pcl::PointXYZ point;
          
          point.x = particles->points[i].x;
          point.y = particles->points[i].y;
          point.z = particles->points[i].z;
          particle_cloud->points.push_back (point);
	}
        
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color (particle_cloud, 250, 99, 71);
          if (!viz.updatePointCloud (particle_cloud, blue_color, name))
            viz.addPointCloud (particle_cloud, blue_color, name);
        }
      }
      /*
      else {
	PCL_WARN("no particles");
      }
      */
    }
  }
  
  void
  drawResult (pcl::visualization::PCLVisualizer& viz)
  {
    char name[1024];
    for (size_t i = 0; i < trackers_.size(); i++) { 

      sprintf (name, "cluster_%d" , int (i));    
      ParticleXYZRPY result = trackers_[i]->getResult ();
      Eigen::Affine3f transformation = trackers_[i]->toEigenMatrix (result);
      // move a little bit for better visualization
      //transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
      RefCloudPtr result_cloud (new RefCloud ());
      pcl::transformPointCloud<RefPointType> ( *references_[i], *result_cloud, transformation);
      {
	pcl::visualization::PointCloudColorHandlerCustom<RefPointType> red_color (result_cloud, 0, 0, 255);
	if (!viz.updatePointCloud (result_cloud, red_color, name))
	  viz.addPointCloud (result_cloud, red_color, name);
      }
    }
  }
  

  void
  removeClusters (size_t clusters_size, pcl::visualization::PCLVisualizer& viz)
  {
    char name[1024];
    for (size_t i = 0; i < clusters_size; i++) {
      sprintf (name, "cluster_%d", int (i));
      viz.removePointCloud (name);
    }
  }

  void
  displayClusters (const pcl::PointCloud<RefPointType>::CloudVectorType &clusters, 
		   pcl::visualization::PCLVisualizer& viz)
  {
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
    
    for (size_t i = 0; i < clusters.size (); i++) {
      sprintf (name, "cluster_%d" , int (i));
      pcl::visualization::PointCloudColorHandlerCustom<PointType> color0(boost::make_shared<pcl::PointCloud<PointType> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
      if (!viz.updatePointCloud (boost::make_shared<pcl::PointCloud<PointType> >(clusters[i]), color0, name))
	viz.addPointCloud (boost::make_shared<pcl::PointCloud<PointType> >(clusters[i]), color0, name);
      viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
      viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
  }

  void
  viz_cb (pcl::visualization::PCLVisualizer& viz)
  {
    boost::mutex::scoped_lock lock (mtx_);
    
    if (new_cloud_)
    {

      CloudPtr cloud = cloud_;
      if (!viz.updatePointCloud (cloud, "object_cloud")) {
	viz.addPointCloud (cloud, "object_cloud");
	viz.resetCameraViewpoint ("object_cloud");
      }
      
      if (have_clusters_) {
	drawParticles (viz);
	drawResult (viz);   
      }
      
      /*
      removeClusters(clusters_size_, viz);      
      displayClusters( clusters_, viz);
      clusters_size_ = clusters_.size();
      */
    }
    new_cloud_ = false;
  }

  void 
  keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.keyUp ()) {
      std::cout << "key: " << event.getKeyCode () << std::endl;
      switch (event.getKeyCode ()) {
      case '1':
	std::cout << "Using [plane comparator]... " << std::endl; 
	mps_.setComparator (plane_comparator_);
	break;
      case '2':
	std::cout << "Using [euclidean comparator]... " << std::endl;
	mps_.setComparator (euclidean_comparator_);
	break;
      case '3':
	std::cout << "Using [rgb comparator]... " << std::endl;
	mps_.setComparator (rgb_comparator_);
	break;
      case '4':
	std::cout << "Using [edge aware comparator]... " << std::endl;
	mps_.setComparator (edge_aware_comparator_);
	break;
      case '5':
	factor_  -= 0.01f;
	std::cout << "factor: " << factor_ << std::endl;
	break;
      case '6':
	factor_  += 0.01f;
	std::cout << "factor: " << factor_ << std::endl;
	break;
      case '7':
	size_  -= 1.0f;
	std::cout << "size: " << size_ << std::endl;
	break;
      case '8':
	size_  += 1.0f;
	std::cout << "size: " << size_ << std::endl;
	break;
      case '9':
	planeMinSize_  -= 1000;
	std::cout << "planeMinSize: " << planeMinSize_ << std::endl;
	break;
      case '0':
	planeMinSize_  += 1000;
	std::cout << "planeMinSize: " << planeMinSize_ << std::endl;
	break;
      case 'v':
	clusterMinSize_  -= 100;
	std::cout << "clusterMinSize: " << clusterMinSize_ << std::endl;
	break;
      case 'b':
	clusterMinSize_  += 100;
	std::cout << "clusterMinSize: " << clusterMinSize_ << std::endl;
	break;
      case 'n':
	angularTh_ -= 0.1f;
	std::cout << "angularTh: " << angularTh_ << std::endl;
	break;
      case 'm':
	angularTh_ += 0.1f;
	std::cout << "angularTh: " << angularTh_ << std::endl;
	break;
      case ',':
	distanceTh_ -= 0.01f;
	std::cout << "distanceTh: " << distanceTh_ << std::endl;
	break;
      case '.':
	distanceTh_ += 0.01f;
	std::cout << "distanceTh: " << distanceTh_ << std::endl;
	break;
      }
    }
  }

  // --------------------------------
  // Tracking functions
  // ----------------------------------------------

  void 
  initTrackers(size_t clusters_size) {
    trackers_.clear();
    
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    // Init a list of tracker objects
    for(size_t i = 0; i < clusters_size; i++) {
      boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker (new ParticleFilterOMPTracker<RefPointType, ParticleT> (8));
      
      tracker->setTrans (Eigen::Affine3f::Identity ());
      tracker->setStepNoiseCovariance (default_step_covariance);
      tracker->setInitialNoiseCovariance (initial_noise_covariance);
      tracker->setInitialNoiseMean (default_initial_mean);
      tracker->setIterationNum (1);
      tracker->setParticleNum (250);
      tracker->setResampleLikelihoodThr(0.00);
      tracker->setUseNormal (false);

      // Setup coherences
      ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
	(new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    
      boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
	= boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
      coherence->addPointCoherence (distance_coherence);
    
      boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
	= boost::shared_ptr<HSVColorCoherence<RefPointType> > (new HSVColorCoherence<RefPointType> ());
      color_coherence->setWeight (0.1);
      coherence->addPointCoherence (color_coherence);

      // Search method used
      //boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
      //boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
      boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);    

      coherence->setSearchMethod (search);
      coherence->setMaximumDistance (0.01);
      tracker->setCloudCoherence (coherence);

      trackers_.push_back(tracker);
    }
  }

  
  void 
  tracking (const RefCloudConstPtr &cloud_ptr)
  {
    //double start = pcl::getTime ();
    FPS_CALC_BEGIN;
    for (size_t i = 0; i < trackers_.size(); i++) {
      trackers_[i]->setInputCloud (cloud_ptr);
      trackers_[i]->compute ();
    }
    //double end = pcl::getTime ();
    FPS_CALC_END("tracking");
    //tracking_time_ = end - start;
  }
  

  // --------------------------------
  // Filter & segmentation functions
  // ----------------------------------------------

  void 
  filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
  {
    FPS_CALC_BEGIN;
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    //pass.setFilterLimits (0.0, 10.0);
    pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimits (0.0, 0.6);
    pass.setKeepOrganized (true);
    pass.setInputCloud (cloud);
    pass.filter (result);
    FPS_CALC_END("filterPassThrough");
  }


  void 
  gridSample (const CloudConstPtr &cloud, Cloud &result, bool use_approx = true, double leaf_size = 0.02)
  {
    FPS_CALC_BEGIN;
    //double start = pcl::getTime ();
    if( use_approx ) {
      pcl::ApproximateVoxelGrid<PointType> grid;
      grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
      grid.setInputCloud (cloud);
      grid.filter (result);
    }
    else {
      pcl::VoxelGrid<PointType> grid;
      grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
      grid.setInputCloud (cloud);
      grid.filter (result);
    }
    //double end = pcl::getTime ();
    //downsampling_time_ = end - start;
    FPS_CALC_END("gridSample");
  }


  void 
  removeZeroPoints (const Cloud &cloud,
		    Cloud &result)
  {
    for (size_t i = 0; i < cloud.points.size (); i++) {
      PointType point = cloud.points[i];
      if (!(fabs(point.x) < 0.01 &&
            fabs(point.y) < 0.01 &&
            fabs(point.z) < 0.01) &&
          !pcl_isnan(point.x) &&
          !pcl_isnan(point.y) &&
          !pcl_isnan(point.z))
        result.points.push_back(point);
    }
  }


  void integralEstimate (const CloudConstPtr &cloud_ptr,
			 pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
			 double factor = 0.02f, // 0.03f
			 double size = 20.0f,   // 10.0
			 double distanceTh = 0.01f ) 
  {
    FPS_CALC_BEGIN;
    nii_.setNormalEstimationMethod (nii_.COVARIANCE_MATRIX); // nii_.AVERAGE_3D_GRADIENT || nii_.COVARIANCE_MATRIX
    nii_.setMaxDepthChangeFactor (factor);
    //nii_.setDepthDependentSmoothing (false);
    nii_.setNormalSmoothingSize (size);
    nii_.setInputCloud (cloud_ptr);
    nii_.compute (*normals_ptr);

    float* distance_map = nii_.getDistanceMap ();
    boost::shared_ptr<pcl::EdgeAwarePlaneComparator<PointType,pcl::Normal> > eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<PointType,pcl::Normal> >(edge_aware_comparator_);
    eapc->setDistanceMap (distance_map);
    eapc->setDistanceThreshold (distanceTh, false);
    FPS_CALC_END("integralEstimate");
  }

  
  void segmentOrganized (const CloudConstPtr &cloud_ptr,
			 const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
			 pcl::PointCloud<RefPointType>::CloudVectorType &clusters,
			 int planeMinSize = 10000,
			 int clusterMinSize = 500,
			 double angularTh = 3.0f,     // degrees (3.0f)
			 double distanceTh = 0.02f )  // (0.02f)
  {
    // Output data structures (use in subsequent processing)
    std::vector<pcl::PlanarRegion<PointType>, Eigen::aligned_allocator<pcl::PlanarRegion<PointType> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;  
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    // Segment planes
    FPS_CALC_BEGIN;   
    mps_.setMinInliers (planeMinSize);
    mps_.setAngularThreshold (pcl::deg2rad (angularTh));
    mps_.setDistanceThreshold (distanceTh);
    mps_.setInputNormals (normals_ptr);
    mps_.setInputCloud (cloud_ptr);
    mps_.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    // Segment objects
    if (regions.size () > 0) {
      std::vector<bool> plane_labels;
      plane_labels.resize (label_indices.size (), false);
      for (size_t i = 0; i < label_indices.size (); i++) {
	if ( (int)label_indices[i].indices.size () > planeMinSize ) {
	  plane_labels[i] = true;
	}
      }

      euclidean_cluster_comparator_->setInputCloud (cloud_ptr);
      euclidean_cluster_comparator_->setLabels (labels);
      euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
      euclidean_cluster_comparator_->setDistanceThreshold ( 0.01f, false); // 0.01f

      pcl::PointCloud<pcl::Label> euclidean_labels;
      std::vector<pcl::PointIndices> euclidean_label_indices;
      pcl::OrganizedConnectedComponentSegmentation<PointType, pcl::Label> seg_ (euclidean_cluster_comparator_);
      seg_.setInputCloud (cloud_ptr);
      seg_.segment (euclidean_labels, euclidean_label_indices);

      for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
	if (euclidean_label_indices[i].indices.size () > clusterMinSize ) {
	  pcl::PointCloud<PointType> cluster;
	  pcl::copyPointCloud ( *cloud_ptr, euclidean_label_indices[i].indices, cluster);
	  clusters.push_back (cluster);
	}    
      }
    }
    FPS_CALC_END("segmentOrganized");    
  }

  // ---------------------------------------
  // Cloud callback function (ROS)
  // -----------------------------------------

  void
  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    boost::mutex::scoped_lock lock (mtx_);
    static int counter = 0;

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    CloudPtr cloud (new Cloud);
    pcl::fromROSMsg (*input, *cloud);

    //std::cout << "Cloud size: " << cloud->width << " x " << cloud->height << std::endl;

    double start = pcl::getTime ();
    FPS_CALC_BEGIN;

    cloud_.reset (new Cloud);
    filterPassThrough (cloud, *cloud_);
 
    if ( counter == 1 ) { 

      PCL_INFO ("segmentation, please wait...\n");
      pcl::PointCloud<pcl::Normal>::Ptr normals_ (new pcl::PointCloud<pcl::Normal>);
      integralEstimate (cloud_, normals_);
      
      pcl::PointCloud<RefPointType>::CloudVectorType clusters;
      segmentOrganized( cloud_, normals_, clusters);
      clusters_ = clusters;
      std::cout << "clusters: " << clusters_.size() << std::endl;

      // Init trackers with segmented cluster clouds
      initTrackers (clusters_.size());
      for (size_t i = 0; i < clusters_.size(); i++) {
	
	RefCloudPtr ref_cloud (new RefCloud);
	removeZeroPoints (clusters_[i], *ref_cloud);

	Eigen::Vector4f cent;
	pcl::compute3DCentroid<RefPointType> (*ref_cloud, cent);
	Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
	trans.translation ().matrix () = Eigen::Vector3f (cent[0], cent[1], cent[2]);
	pcl::transformPointCloud<RefPointType> (*ref_cloud, *ref_cloud, trans.inverse());

	CloudPtr reduced_cloud (new Cloud);
	gridSample ( ref_cloud, *reduced_cloud, false);
	trackers_[i]->setReferenceCloud (reduced_cloud);
	trackers_[i]->setTrans (trans);
	references_.push_back(ref_cloud);
	trackers_[i]->setMinIndices (int (ref_cloud->size ()) / 2);      
      }
      have_clusters_ = true;
    }
    else {
      if (have_clusters_) {
	CloudPtr reduced_cloud (new Cloud);
	gridSample ( cloud_, *reduced_cloud);
	tracking (reduced_cloud);
      }
    }
    counter++;
    new_cloud_ = true;
    double end = pcl::getTime ();
    //computation_time_ = end - start;
    FPS_CALC_END("computation");
    //counter_++;
  }

  void 
  spin() {
    while (ros::ok()) {
      ros::spin();
    }  
  }

// Private
private:

  // ROS variables
  ros::Subscriber sub_;
  ros::NodeHandle nh_; 

  pcl::visualization::CloudViewer viewer_;
  CloudPtr cloud_, cloud_ref_;
  boost::mutex mtx_;
  bool new_cloud_;

  pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> nii_;
  pcl::OrganizedMultiPlaneSegmentation<PointType, pcl::Normal, pcl::Label> mps_;    

  // Comparator objects
  pcl::PlaneCoefficientComparator<RefPointType, pcl::Normal>::Ptr plane_comparator_;
  pcl::EuclideanPlaneCoefficientComparator<RefPointType, pcl::Normal>::Ptr euclidean_comparator_;
  pcl::RGBPlaneCoefficientComparator<RefPointType, pcl::Normal>::Ptr rgb_comparator_;
  pcl::EdgeAwarePlaneComparator<RefPointType, pcl::Normal>::Ptr edge_aware_comparator_;
  pcl::EuclideanClusterComparator<RefPointType, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

  //Segment Objects
  pcl::PointCloud<RefPointType>::CloudVectorType clusters_;  
  size_t clusters_size_;
  bool have_clusters_;
 
  // Tracker
  std::vector< boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > > trackers_;
  std::vector<CloudPtr> references_;

  // Variables
  double factor_;
  double size_;
  int planeMinSize_;
  int clusterMinSize_;
  double angularTh_;
  double distanceTh_;


  /*
  pcl::visualization::CloudViewer viewer_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  CloudPtr cloud_pass_;
  CloudPtr cloud_pass_downsampled_;
  CloudPtr plane_cloud_;
  CloudPtr nonplane_cloud_;
  CloudPtr cloud_hull_;
  CloudPtr segmented_cloud_;
  CloudPtr reference_;
  std::vector<pcl::Vertices> hull_vertices_;
  
  boost::mutex mtx_;
  bool new_cloud_;
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
  boost::shared_ptr<ParticleFilter> tracker_;
  int counter_;
  bool use_convex_hull_;
  bool visualize_non_downsample_;
  bool visualize_particles_;
  double tracking_time_;
  double computation_time_;
  double downsampling_time_;
  double downsampling_grid_size_;
  */
};

// Main 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_viewer_node");
  ros::NodeHandle nh;

  ObjectViewer<pcl::PointXYZRGBA> node(nh);
  node.spin();
  return 0;
}
