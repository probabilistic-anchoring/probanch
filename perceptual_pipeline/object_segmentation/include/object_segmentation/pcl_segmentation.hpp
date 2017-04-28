#ifndef __SEGMENTATION_HPP__
#define __SEGMENTATION_HPP__

#include <pcl/common/pca.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h> 
#include <pcl/filters/extract_indices.h>

// PCL normals
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

// PCL segmentation (Organized style)
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

// PCL comparators
#include <pcl/segmentation/comparator.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate ("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

namespace segmentation {

  // Used namspaces
  using namespace std;

  // Typedefines
  typedef pcl::PointXYZRGBA Point;  // pcl::PointXYZ OR pcl::PointXYZRGB OR pcl::PointXYZRGBA
  typedef pcl::PointXYZ SimplePoint;
  typedef pcl::search::KdTree<Point>::Ptr KdTreePtr;


  // -------------------
  // Segmentation class defintion
  // -----------------------------
  class Segmentation {
  public:

    // Constructor and destructor
    Segmentation();
    ~Segmentation() {}

    // Public functions
    void clusterClassic( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			 vector<pcl::PointIndices> &cluster_indices );
    void clusterOrganized( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			   vector<pcl::PointIndices> &cluster_indices );
    void clusterLccp( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
		      vector<pcl::PointIndices> &cluster_indices );  // Not implemented at the moment!
    void postProcess(pcl::PointCloud<Point>::Ptr &cloud_ptr, vector<pcl::Vertices> &indices);

    // Setter functions for segmentation parameters
    void setComparatorType (int type);
    void setPlaneMinSize (int size) { this->plane_min_size_ = size; }
    void setClusterMinSize (int size) { this->cluster_min_size_ = size; }
    void setAngularTh (double th) { this->angular_th_ = th; }
    void setDistanceTh (double th) { this->distance_th_ = th; }
    void setRefineFactor (double factor) { if( factor >= 0.0 && factor <= 2.0 ) { this->refine_factor_ = factor; } }

  private:

    // Segmentation
    pcl::IntegralImageNormalEstimation<Point, pcl::Normal> niie_;
    pcl::OrganizedMultiPlaneSegmentation<Point, pcl::Normal, pcl::Label> mps_;

    // Comparator objects
    pcl::PlaneCoefficientComparator<Point, pcl::Normal>::Ptr plane_comparator_;
    pcl::EuclideanPlaneCoefficientComparator<Point, pcl::Normal>::Ptr euclidean_comparator_;
    pcl::RGBPlaneCoefficientComparator<Point, pcl::Normal>::Ptr rgb_comparator_;
    pcl::EdgeAwarePlaneComparator<Point, pcl::Normal>::Ptr edge_aware_comparator_;
    pcl::EuclideanClusterComparator<Point, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;
    std::vector<float> distance_map_;
  
    // Private variables and objects
    KdTreePtr tree_;
    pcl::ExtractIndices<Point> extract_p_;
    pcl::ExtractIndices<pcl::Normal> extract_n_; 

    // Private segmentation parameters 
    int plane_min_size_;    // 10000
    int cluster_min_size_;  // 500  
    double angular_th_;     // 3.0f (degres)
    double distance_th_;    // 0.02f (meters)
    double refine_factor_;  // 1.0f (sample refinment factor)

    // Private filter and sampling functions
    void outlierFilter( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			pcl::PointCloud<Point>::Ptr &filtered_ptr,
			int meanK = 50,
			double th = 1.0 );  
    void downsample( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
		     pcl::PointCloud<Point>::Ptr &downsampled_ptr,
		     double leafSize = 0.01f );
    void reconstruct( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
		      pcl::PointCloud<Point>::Ptr &smoothed_ptr,
		      pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
		      double radius = 0.03f,
		      bool polynomialFit = true );
    
    // Private normals estimation functions
    void treeEstimate( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
		       pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
		       int kNN = 10,
		       double radius = 0.03f );
    void integralEstimate( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			   pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
			   double factor = 0.02f, // 0.02f
			   double size = 20.0f ); // 10.0f

    // Private segmentation functions
    void segmentPlane( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
		       const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
		       pcl::PointCloud<Point>::Ptr &segmented_cloud_ptr,
		       pcl::PointCloud<pcl::Normal>::Ptr &segmented_normals_ptr,
		       int ite = 100,
		       double weight = 0.1f,
		       double th = 0.05f);
    void segmentClusters( const pcl::PointCloud<Point>::Ptr &objects_ptr,
			  vector<pcl::PointIndices> &cluster_indices,
	   		  double tolerance = 0.03f,
			  int clusterSize = 100 );

    void segmentOrganized( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			   const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
			   vector<pcl::PointIndices> &cluster_indices,
			   int planeMinSize,
			   int clusterMinSize,
			   double angularTh,    // degrees (3.0f)
			   double distanceTh,  // (0.02f)
			   double factor );      // Simple refinment factor
    
    void segmentLCCP( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
		      const pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr,
		      vector<pcl::PointIndices> &cluster_indices,
		      float voxelResolution = 0.0075f,
		      float seedResolution = 0.03f,
		      float colorImp = 0.0f,
		      float spatialImp = 1.0f,
		      float normalImp = 4.0f,
		      int concavityTh = 10,
		      float smoothnessTh = 0.1 );
  };

  
  // -------------------
  // Change detection class defintion
  // ----------------------------------
  class ChangeDetection {

    // Type define octree change detector
    typedef pcl::octree::OctreePointCloudChangeDetector<Point> OcttreeChangeDetector;
    typedef boost::shared_ptr<OcttreeChangeDetector> OcttreeChangeDetectorPtr;

    // Change detector object
    OcttreeChangeDetectorPtr octree_; 
    bool initalized_;
  public:
    
    // Constructor and destructor
    ChangeDetection();
    ~ChangeDetection() {}
    
    // Detect changes between two clouds
    void detect(const pcl::PointCloud<Point>::Ptr &cloud_ptr, vector<int> &indices);
  private:
  };
  
  // -------------------
  // Namespace functions
  // -------------------
  void passThroughFilter( const pcl::PointCloud<Point>::Ptr &cloud_ptr,
			  pcl::PointCloud<Point>::Ptr &filtered_ptr,
			  const string axis,
			  double min,
			  double max,
			  bool keep_organized = true );
  void getPosition( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
		    geometry_msgs::Pose &pos );
  void getShape( const pcl::PointCloud<Point>::Ptr &projected_cloud_ptr,
		 geometry_msgs::Point &dim );
}

#endif // __SEGMENTATION_HPP__
