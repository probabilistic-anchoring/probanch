#ifndef __FEATURES_HPP__
#define __FEATURES_HPP__

#include <cmath> 
#include <iostream>
#include <vector>

// OpenCV includes
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#if CV_MAJOR_VERSION == 2 // opencv2 only
#include <opencv2/features2d/features2d.hpp>
#endif

#include <ml/ml.hpp>

#include <features/shades.hpp>

namespace visual_2d {

  // ---[ Used namespaces ]---
  using namespace std;
  using namespace cv;
  
  // ---[ Class for color features
  // ----------------------------------------
  class ColorFeatures {
  public:

    // --[ Constructor(s)/destructor ]-- 
    ColorFeatures(const string &filename);
    ColorFeatures(int hbins = 180, int sbins = 256, int vbins = 256);
    ~ColorFeatures() {}

    // --[ Color detection ]--
    void load(const string &filename);
    void calculate( const Mat &img,
		    Mat &hist,
		    const Mat &mask = Mat() );

    void predict( const Mat &hist,
		  vector<float> &preds);

    void normalize( Mat &hist );

    void reduce( const Mat &hist,
		 Mat &result, 
		 int hbins = 30, int sbins = 32, int vbins = 4 );

    string colorSymbol(int idx);
    Scalar getColor(int pred);

    // Static helper functions
    static Mat equalizeIntensity(const Mat& img);

    // Getter/setter functions
    void setScaleFactor(float val) { if( val >= 0.0 && val <= 1.0 ) this->_scale = val; }
    float getScaleFactor() { return this->_scale; }
  
  private:

    // SVM classifier
    machine::MachinePtr _svm; 
  
    int _hbins;
    int _sbins;
    int _vbins;

    // Scale factor (in range: [0.0, 1.0])
    float _scale;

    // --[ Private helper function ]--
    void buildTrainingData( const Color index[], 
			    int n,
			    float cl,
			    Mat &trainData,
			    Mat &labelData );
    float totalValue(const Mat &hist);
    float maxValue(const Mat &hist);
    
  }; // ...end of class. ]---

} // namespace 'visual_2d'



#if CV_MAJOR_VERSION == 2 // opencv2 only
// --------------------------
// Class for handling keypoint features
// ----------------------------------------
class KeypointFeatures {
public:

  // Different feature derectors
  enum FeatureDetectorType {
    CV_BRIEF = 0,
    CV_ORB   = 1,
    CV_BRISK = 2,
    CV_FREAK = 3  // Need to update to OpenCV v. 2.4.3
  };

  // -------------------
  // Public function
  // -------------------
  KeypointFeatures( int numKeyPoints = 2500,
	    FeatureDetectorType detectorType = CV_BRISK );
  ~KeypointFeatures();

  // Feature extracting and detecting
  void detect( const cv::Mat &img, std::vector<cv::KeyPoint> &keypoints);
  cv::Mat extract( const cv::Mat &img, 
		   std::vector<cv::KeyPoint> &keypoints,
		   int n = 2000 );

  // Feature matching (not used at the monment)
  void match( const cv::Mat &queryDescriptor, 
	      const cv::Mat &trainDescriptor,
	      std::vector<cv::DMatch> &matches,
	      bool symmetrical = false);
  void match( const cv::Mat &queryDescriptor, 
	      const std::vector<cv::Mat> &trainDescriptors,
	      std::vector<cv::DMatch> &matches);

  
  // For later feature to text mapping
  std::string getDetectorStr() { return this->_detectorStr; }
  int getDescriptorLevel() { return this->_descriptorLevel; }

  // ------------------------------------------
  // Public static helper functions
  // -----------------------------------------
  static void filterMatches( std::vector<std::vector<cv::DMatch> > &matches,
			     std::vector<cv::DMatch> &filtered,
			     bool bestMatches = true,
			     float ratio = 0.8f);

  
  static cv::Mat filterResponseN( std::vector<cv::KeyPoint> &keypoints,
				  cv::Mat &descriptor,
				  int n = 500 );

  static void filterSymmetrical( const std::vector<cv::DMatch> &matches1,
				 const std::vector<cv::DMatch> &matches2,
				 std::vector<cv::DMatch>& result );

  static cv::Mat filterDescriptor( const cv::Mat& descriptor,
				   const std::vector<int>& index,
				   int type );

// Private space
private:

  // Feature detector object
  cv::Ptr<cv::FeatureDetector> _detector;

  // Descriptor extractor object
  cv::Ptr<cv::DescriptorExtractor> _extractor;

  // Matcher object
  cv::Ptr<cv::DescriptorMatcher> _matcher;

  // Feature detector parameters
  int _numKeyPoints;  // Number of keypoints

  // Feature information
  std::string _detectorStr;
  int _descriptorLevel;

}; 
#endif // opencv2 ...


#endif // __VISUAL_HPP__
