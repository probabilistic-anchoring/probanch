#include <string>
#include <ros/ros.h>

// PCL filter includes
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

// PCL segmentation (Euclidean style) 
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>

// PCL common
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

/*
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>  // <-- Requires PCL 1.8
*/

#include <object_segmentation/pcl_segmentation.hpp>

// -------------------
// Segmentation class functions
// ------------------------------
namespace segmentation {

  // Constructor
  Segmentation::Segmentation() :
    plane_min_size_(10000), 
    cluster_min_size_(500), 
    angular_th_(3.0),
    distance_th_(0.02),
    refine_factor_(1.0)
  {
    // Create a KD-Tree
    this->tree_ = boost::make_shared<pcl::search::KdTree<Point> >();

    // Create comparator objects
    plane_comparator_.reset (new pcl::PlaneCoefficientComparator<Point, pcl::Normal> ());
    euclidean_comparator_.reset (new pcl::EuclideanPlaneCoefficientComparator<Point, pcl::Normal> ());
    rgb_comparator_.reset (new pcl::RGBPlaneCoefficientComparator<Point, pcl::Normal> ());
    edge_aware_comparator_.reset (new pcl::EdgeAwarePlaneComparator<Point, pcl::Normal> ());
    euclidean_cluster_comparator_ = 
      pcl::EuclideanClusterComparator<Point, pcl::Normal, pcl::Label>::Ptr 
      (new pcl::EuclideanClusterComparator<Point, pcl::Normal, pcl::Label> ());

    // Set default compartor type (Euclidean comparator)
    this->setComparatorType(1);
  }

  // Clustering functions
  void Segmentation::clusterClassic(const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				    vector<pcl::PointIndices> &cluster_indices) {

    // Saftey check
    if( cloud_ptr->points.empty() ) {
      ROS_INFO("[Segmentation::cluster] No valid points in scene -- cloud is empty");
      return;
    }

    // Filter and downsample the cloud
    pcl::PointCloud<Point>::Ptr filtered_ptr (new pcl::PointCloud<Point>);
    this->outlierFilter( cloud_ptr, filtered_ptr);
    pcl::PointCloud<Point>::Ptr downsampled_ptr (new pcl::PointCloud<Point>);
    this->downsample( filtered_ptr, downsampled_ptr);
    ROS_INFO("Downsampled cloud: %d", (int)downsampled_ptr->points.size());  

    // Estimate the surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
    this->treeEstimate( downsampled_ptr, normals_ptr );
    //this->reconstruct( _cloud_ptr, _cloud_ptr, _normals_ptr ); // Smooth the surface and estimate normals
    
    // Detect and remove the "table" plane
    this->segmentPlane( downsampled_ptr, normals_ptr, downsampled_ptr, normals_ptr);
    ROS_INFO("[Segmentation::clusterClassic] Cloud without plane: %d", (int)downsampled_ptr->points.size());  

    // Cluster remaining points
    this->segmentClusters( downsampled_ptr, cluster_indices);
    //ROS_INFO("[Segmentation::clusterClassic] Clusters: %d", (int)cluster_indices.size());
  }
  
  void Segmentation::clusterOrganized(const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				      vector<pcl::PointIndices> &cluster_indices) {

    // Saftey check 
    if( cloud_ptr->isOrganized ()) {

      //FPS_CALC ("organized clustering");

      // Estimate the surface normals
      pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
      this->integralEstimate( cloud_ptr, normals_ptr );
      
      // Cluster the point cloud
      this->segmentOrganized( cloud_ptr, 
			      normals_ptr, 
			      cluster_indices, 
			      this->plane_min_size_, 
			      this->cluster_min_size_, 
			      this->angular_th_,
			      this->distance_th_,
			      this->refine_factor_ );
      //ROS_INFO("[Segmentation::clusterOrganized] Clusters: %d", (int)cluster_indices.size());
    }
    else {
      ROS_WARN("[Segmentation::clusterOrganized] Trying to perform organized segmentation with un-oranized point cloud data!");  
    }
  }

  void Segmentation::clusterLccp(const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				  vector<pcl::PointIndices> &cluster_indices) {
    
    // Estimate the surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_ptr (new pcl::PointCloud<pcl::Normal>);
    this->integralEstimate( cloud_ptr, normals_ptr );

    // Cluster the point cloud
    this->segmentLCCP( cloud_ptr, normals_ptr, cluster_indices);
    //ROS_INFO("[Segmentation::clusterLccp] Clusters: %d", (int)cluster_indices.size());
  }

  void Segmentation::postProcess(pcl::PointCloud<Point>::Ptr &cloud_ptr, vector<pcl::Vertices> &indices) {
     
    // Calculating the convex hull of the cluster 
    // (and indcies of uttermoset points of the hull)
    pcl::ConvexHull<Point> hull_;
    hull_.setInputCloud (cloud_ptr);
    std::vector<pcl::Vertices> result;
    hull_.reconstruct (*cloud_ptr, indices);
  }

  // Set comparator used for refinement
  void Segmentation::setComparatorType (int type) {
    switch(type) {
    case 0:
      mps_.setComparator (plane_comparator_);
      ROS_INFO("[Segmentation::setComparatorType] Using 'plane comparator' for organized segmentation.");
      break;
    case 1:
      mps_.setComparator (euclidean_comparator_);
      ROS_INFO("[Segmentation::setComparatorType] Using 'Euclidean comparator' for organized segmentation.");
      break;
    case 2:
      mps_.setComparator (rgb_comparator_); 
      ROS_INFO("[Segmentation::setComparatorType] Using 'RGB comparator' for organized segmentation.");
      break;
    default:
      mps_.setComparator (edge_aware_comparator_);
      ROS_INFO("[Segmentation::setComparatorType] Using 'edge aware comparator' for organized segmentation.");
      break;
    }
  }

  
  /* _____________ Filter ans sampling functions ___________________ */


  // Filter and remove remove outliers
  void Segmentation::outlierFilter( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				    pcl::PointCloud<Point>::Ptr &filtered_ptr,
				    int meanK,
				    double th ) {
    pcl::PointCloud<Point>::Ptr result_ptr (new pcl::PointCloud<Point>);
    pcl::StatisticalOutlierRemoval<Point> sor_;
    sor_.setInputCloud (cloud_ptr);
    sor_.setMeanK (meanK);
    sor_.setStddevMulThresh (th);
    sor_.filter (*result_ptr);
    filtered_ptr.swap (result_ptr);
  }


  // Downsample a point cloud using a voxel grid filter
  void Segmentation::downsample( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				 pcl::PointCloud<Point>::Ptr &downsampled_ptr,
				 double leafSize ) {
    pcl::PointCloud<Point>::Ptr result_ptr (new pcl::PointCloud<Point>);
    pcl::VoxelGrid<Point> grid_;
    grid_.setLeafSize( leafSize, leafSize, leafSize);
    grid_.setDownsampleAllData(false); 
    grid_.setInputCloud(cloud_ptr);
    grid_.filter(*result_ptr);
    downsampled_ptr.swap (result_ptr);
  }

  // Reconstruct a point cloud (can be used to smooth and resample noisy data)
  void Segmentation::reconstruct( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				  pcl::PointCloud<Point>::Ptr &smoothed_ptr,
				  pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
				  double radius,
				  bool polynomialFit ) {
    pcl::PointCloud<pcl::PointNormal>::Ptr result_ptr (new  pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares< Point, pcl::PointNormal> mls_;
    mls_.setComputeNormals (true);
    mls_.setInputCloud (cloud_ptr);
    mls_.setPolynomialFit (polynomialFit);
    mls_.setSearchMethod (this->tree_);
    mls_.setSearchRadius (radius);
    mls_.process (*result_ptr);   // Reconstruct
    
    // Assign to output (copied by the template type)
    pcl::copyPointCloud( *result_ptr, *smoothed_ptr);
    pcl::copyPointCloud( *result_ptr, *normals_ptr);
  }
  
  
  /* _____________ Normals estimation functions ___________________ */
  
  // Estimate 3d normals based on a kd-tree representation
  void Segmentation::treeEstimate( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				   pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
				   int kNN,
				   double radius ) {
    //pcl::NormalEstimation<Point, pcl::Normal> n3d_;
    pcl::NormalEstimationOMP<Point, pcl::Normal> n3d_(8); // Multi-threaded with 8 threads (set 0 for automatic)
    n3d_.setSearchMethod(this->tree_);
    //n3d_.setKSearch(kNN);  
    n3d_.setRadiusSearch(radius); 
    n3d_.setInputCloud(cloud_ptr);
    n3d_.compute(*normals_ptr);
  }
  
  // Estimate 3d normals based on integral images 
  void Segmentation::integralEstimate( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				       pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
				       double factor,
				       double size ) {

    // Set the parameters for normal estimation
    niie_.setNormalEstimationMethod (niie_.COVARIANCE_MATRIX); // nii_.AVERAGE_3D_GRADIENT || nii_.COVARIANCE_MATRIX
    niie_.setMaxDepthChangeFactor (factor);
    niie_.setNormalSmoothingSize (size);

    // Estimate normals in the cloud
    niie_.setInputCloud (cloud_ptr);
    niie_.compute (*normals_ptr);
    
    // Assign a distance map to the edge aware comparator (later used to refine the segmentation)
    float* map = niie_.getDistanceMap ();
    //distance_map_.assign(map, map + cloud_ptr->size() ); // ...must copy the data out
    edge_aware_comparator_->setDistanceMap(map);
    edge_aware_comparator_->setDistanceThreshold (0.01f, true);    
  }


  /* _____________ Segmentation functions ___________________ */
  
  // Plane model segmentation (detect table plane)
  void Segmentation::segmentPlane( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				   const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
				   pcl::PointCloud<Point>::Ptr &segmented_cloud_ptr,
				   pcl::PointCloud<pcl::Normal>::Ptr &segmented_normals_ptr,
				   int ite,
				   double weight,
				   double th ) {
    
    // Create segmentation object and set all parameters
    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
    seg_.setMaxIterations(ite);
    seg_.setNormalDistanceWeight(weight);
    seg_.setDistanceThreshold(th); 
    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices); 
    pcl::ModelCoefficients::Ptr coefficients_ptr(new pcl::ModelCoefficients); 
    
    seg_.setInputCloud(cloud_ptr); 
    seg_.setInputNormals(normals_ptr);
    seg_.segment(*inliers_ptr, *coefficients_ptr);
  
    // Extract the planar inliers from the input cloud
    pcl::PointCloud<Point>::Ptr result_p_ptr (new pcl::PointCloud<Point>);
    pcl::PointCloud<pcl::Normal>::Ptr result_n_ptr (new pcl::PointCloud<pcl::Normal>);
    this->extract_p_.setInputCloud (cloud_ptr);
    this->extract_n_.setInputCloud (normals_ptr);
    this->extract_p_.setIndices (inliers_ptr);
    this->extract_n_.setIndices (inliers_ptr);
    this->extract_p_.setNegative (true);
    this->extract_n_.setNegative (true);
    this->extract_p_.filter (*result_p_ptr);
    this->extract_n_.filter (*result_n_ptr);  
    segmented_cloud_ptr.swap (result_p_ptr);
    segmented_normals_ptr.swap (result_n_ptr);
  }
  
  // Create objects clusters (based on  segmented indices)
  void Segmentation::segmentClusters( const pcl::PointCloud<Point>::Ptr &objects_ptr,
				      vector<pcl::PointIndices> &cluster_indices,
				      double tolerance,
				      int clusterSize ) {
    // Extract cluster indices
    pcl::EuclideanClusterExtraction<Point> euclidean_;
    euclidean_.setClusterTolerance (tolerance);
    euclidean_.setMinClusterSize (clusterSize);
    euclidean_.setSearchMethod (this->tree_);
    euclidean_.setInputCloud (objects_ptr);
    euclidean_.extract (cluster_indices);    
  }
  
  // Create object clusters based on organized multi-plane segmentation 
  void Segmentation::segmentOrganized( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				       const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
				       vector<pcl::PointIndices> &cluster_indices,
				       int planeMinSize,
				       int clusterMinSize,
				       double angularTh,    
				       double distanceTh,
				       double factor ) {

    // Output data structures (use in subsequent processing)
    std::vector<pcl::PlanarRegion<Point>, Eigen::aligned_allocator<pcl::PlanarRegion<Point> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;  
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;    
    
    // Refinemanet comperator
    pcl::PlaneRefinementComparator<Point, pcl::Normal, pcl::Label>::Ptr refinement_compare_ (new pcl::PlaneRefinementComparator<Point, pcl::Normal, pcl::Label>());
    refinement_compare_->setDistanceThreshold (distanceTh * factor, false);

    // Plane segmentation
    mps_.setRefinementComparator (refinement_compare_);  // Need to be set
    mps_.setMinInliers (planeMinSize);
    mps_.setAngularThreshold (pcl::deg2rad (angularTh));
    mps_.setDistanceThreshold (distanceTh);
    mps_.setMaximumCurvature (0.001); // a small curvature
    mps_.setProjectPoints (true);

    // Segment plane(s)
    mps_.setInputNormals (normals_ptr);
    mps_.setInputCloud (cloud_ptr);
    mps_.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    //Segment Objects
    if (regions.size () > 0) {
      
      std::vector<bool> plane_labels;
      plane_labels.resize (label_indices.size (), false);
      for (size_t i = 0; i < label_indices.size (); i++) {
	if ( (int)label_indices[i].indices.size () > planeMinSize ) {
	  plane_labels[i] = true;
	}
      }
      
      // Euclidean clustering 
      euclidean_cluster_comparator_->setInputCloud (cloud_ptr);
      euclidean_cluster_comparator_->setLabels (labels);
      euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
      euclidean_cluster_comparator_->setDistanceThreshold ( distanceTh * factor, false); // 0.01f

      pcl::PointCloud<pcl::Label> euclidean_labels;
      std::vector<pcl::PointIndices> euclidean_indices;
      pcl::OrganizedConnectedComponentSegmentation<Point, pcl::Label> seg_ (euclidean_cluster_comparator_);
      seg_.setInputCloud (cloud_ptr);
      seg_.segment (euclidean_labels, euclidean_indices);
      for (size_t i = 0; i < euclidean_indices.size (); i++) {
	if ( (int)euclidean_indices[i].indices.size () > clusterMinSize && (int)euclidean_indices[i].indices.size () < planeMinSize ) {
	  cluster_indices.push_back(euclidean_indices[i]);
	}    
      }
    }
  }

  // Create object clusters based on partitioning a supervoxel graph into groups of locally convex...
  // ... connected supervoxels separated by concave borders.
  void Segmentation::segmentLCCP( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				  const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
				  vector<pcl::PointIndices> &cluster_indices,
				  float voxelResolution,
				  float seedResolution,
				  float colorImp,
				  float spatialImp,
				  float normalImp,
				  int concavityTh,
				  float smoothnessTh ) {
    /*
    // Output data structures (use in subsequent processing)
    std::map<uint32_t, pcl::Supervoxel<Point>::Ptr> supervoxel_clusters;
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    // Inital supervoxel oversegmentation
    pcl::SupervoxelClustering<Point> super_ (voxelResolution, seedResolution);
    super_.setUseSingleCameraTransform (false); // ...or true?
    super_.setInputCloud (cloud_ptr);
    super_.setNormalCloud (normals_ptr);
    super_.setColorImportance (colorImp);
    super_.setSpatialImportance (spatialImp);
    super_.setNormalImportance (normalImp);
    super_.extract (supervoxel_clusters);
    // Refine supervoxels
    super_.refineSupervoxels (2, supervoxel_clusters);
    // Get supervoxel adjacency 
    super_.getSupervoxelAdjacency (supervoxel_adjacency);
    // Segmentation LCCP 
    pcl::LCCPSegmentation<Point> lccp_;
    lccp_.setConcavityToleranceThreshold (concavityTh);
    lccp_.setSanityCheck (false); // ...or true?
    lccp_.setSmoothnessCheck (true, voxelResolution, seedResolution, smoothnessTh);
    lccp_.setKFactor (0); // ... or 1?
    lccp_.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
    lccp_.setMinSegmentSize (0);
    lccp_.segment ();
    // Get labeled clouds
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = super_.getLabeledCloud (); 
    lccp_.relabelCloud (*lccp_labeled_cloud);
    // Create the Output
    if ( lccp_labeled_cloud->size () == input_cloud_ptr->size () ) {
    }
    */
  }
}


// -------------------
// Change detection class functions
// ----------------------------------
namespace segmentation {

  // Constructor
  ChangeDetection::ChangeDetection() : initalized_(false) {

    // Create a change detector object with a resolution of 128
    octree_.reset (new OcttreeChangeDetector (128.0f));
  }

  // Function for detecting changes between two clouds - return indcies of movement points
  void ChangeDetection::detect(const pcl::PointCloud<Point>::Ptr &cloud_ptr, std::vector<int> &indices) {

    // Check if we have a history...
    if( !initalized_ ) {

      // ...if not, just add points to octree
      octree_->setInputCloud (cloud_ptr);
      octree_->addPointsFromInputCloud ();
      initalized_ = true;
      return;
    }

    // Switch octree buffers
    octree_->switchBuffers ();
    
    // Add new points to octree
    octree_->setInputCloud (cloud_ptr);
    octree_->addPointsFromInputCloud ();

    // Get vector of point indices from octree voxels 
    octree_->getPointIndicesFromNewVoxels (indices);
  }
}


// -------------------
// Namespace functions
// -------------------

// Filter the a cloud based on a pass through filter
void segmentation::passThroughFilter( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
				      pcl::PointCloud<Point>::Ptr &filtered_ptr,
				      const string axis,
				      double min,
				      double max,
				      bool keep_organized ) {
    pcl::PointCloud<Point>::Ptr result_ptr (new pcl::PointCloud<Point>);
    pcl::PassThrough<Point> pass_;
    pass_.setInputCloud (cloud_ptr); 
    pass_.setFilterFieldName (axis);
    pass_.setFilterLimits ( min, max);  
    pass_.setKeepOrganized (keep_organized);
    //pass_.setFilterLimitsNegative (true);
    pass_.filter (*result_ptr);
    //filtered_ptr = result_ptr;
    filtered_ptr.swap (result_ptr);
  }


void segmentation::getPosition( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
				geometry_msgs::Pose &pos ) {
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cloud_ptr, centroid);

  pos.position.x = centroid[0];
  pos.position.y = centroid[1];
  pos.position.z = centroid[2];
  //std::cout << "Location: [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "]" << std::endl;

  pos.orientation.x = 0.0;
  pos.orientation.y = 0.0;
  pos.orientation.z = 0.0;
  pos.orientation.w = 1.0;
}

void segmentation::getOrientedPosition( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
					geometry_msgs::Pose &pos ) {
  pcl::PCA<Point> pca;
  pca.setInputCloud(cloud_ptr);

  // Calculate the position and the orientation
  Eigen::Vector4f centroid = pca.getMean();
  Eigen::Quaternionf rotation = Eigen::Quaternionf(pca.getEigenVectors());

  pos.position.x = centroid[0];
  pos.position.y = centroid[1];
  pos.position.z = centroid[2];
  //std::cout << "Location: [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "]" << std::endl;

  pos.orientation.x = rotation.x();
  pos.orientation.y = rotation.y();
  pos.orientation.z = rotation.z();
  pos.orientation.w = rotation.w();
}

void segmentation::getSize( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			     geometry_msgs::Vector3 &shape ) {
  // Get the 3D bounding box
  Point proj_min;
  Point proj_max;
  pcl::getMinMax3D (*cloud_ptr, proj_min, proj_max); 

  double width = fabs(proj_max.x - proj_min.x);
  double height = fabs(proj_max.y - proj_min.y);
  double depth = fabs(proj_max.z - proj_min.z); 

  shape.x = width;
  shape.y = height;
  shape.z = depth;
  //std::cout << "Shape: [" << width << ", " << height << ", " << depth << "]" << std::endl;
}
