#ifndef __SEGMENTATION_HPP__
#define __SEGMENTATION_HPP__

#include <pcl/common/pca.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h> 
#include <pcl/filters/extract_indices.h>

// PCL comparators
#include <pcl/segmentation/comparator.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

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
    ros::NodeHandle _nh;

    // Constructor and destructor
    Segmentation(const pcl::PointCloud<Point>::Ptr &cloud_ptr);
    ~Segmentation() {}

    // Public functions
    void cluster_classic(vector<pcl::PointIndices> &cluster_indices);
    void cluster_organized(vector<pcl::PointIndices> &cluster_indices, int type = 0);
    void cluster_lccp(vector<pcl::PointIndices> &cluster_indices);  
    void post_process(pcl::PointCloud<Point>::Ptr &cloud_ptr, vector<pcl::Vertices> &indices);

  private:

    // Comparrator objects
    pcl::PlaneCoefficientComparator<Point, pcl::Normal>::Ptr plane_comparator_;
    pcl::EuclideanPlaneCoefficientComparator<Point, pcl::Normal>::Ptr euclidean_comparator_;
    pcl::RGBPlaneCoefficientComparator<Point, pcl::Normal>::Ptr rgb_comparator_;
    pcl::EdgeAwarePlaneComparator<Point, pcl::Normal>::Ptr edge_aware_comparator_;
  
    // Private variables and objects
    pcl::PointCloud<Point>::Ptr _cloud_ptr;
    pcl::PointCloud<pcl::Normal>::Ptr _normals_ptr;
    KdTreePtr _tree;
    pcl::ExtractIndices<Point> _extract_p;
    pcl::ExtractIndices<pcl::Normal> _extract_n; 

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
			   int comparatorType = 3,
			   int planeMinSize = 20000,
			   int clusterMinSize = 500,
			   double angularTh = 3.0f,     // degrees (3.0f)
			   double distanceTh = 0.02f );  // (0.02f)
    
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
  void getLocation( const pcl::PointCloud<Point>::Ptr &cloud_ptr, 
		    geometry_msgs::Pose &pos );
  void getShape( const pcl::PointCloud<Point>::Ptr &projected_cloud_ptr,
		 geometry_msgs::Point &dim );
}

#endif // __SEGMENTATION_HPP__
