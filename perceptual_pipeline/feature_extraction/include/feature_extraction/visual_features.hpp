#ifndef __FEATURES_HPP__
#define __FEATURES_HPP__

#include <cmath> 
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>

#include <feature_extraction/colors.hpp>

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


// --------------------------
// Class for color features
// ----------------------------------------
class ColorFeatures {
public:

  // -------------------
  // Public function
  // -------------------
  ColorFeatures(int hbins = 180, int sbins = 256, int vbins = 256);
  ~ColorFeatures();

  // Color detection
  void calculate( const cv::Mat &img,
		  cv::Mat &hist,
		  const cv::Mat &mask = cv::Mat() );

  void predict( const cv::Mat &hist,
		std::vector<float> &preds);

  void normalize( cv::Mat &hist );

  std::string colorSymbol(int idx);

  // Static helper functions
  static cv::Mat equalizeIntensity(const cv::Mat& img);

// Private space
private:

  // SVM classifier
  cv::Ptr<CvSVM> _svm;
  int _hbins;
  int _sbins;
  int _vbins;

  // Private helper function
  void buildTrainingData( const Color index[], 
			  int n,
			  float cl,
			  cv::Mat &trainData,
			  cv::Mat &labelData );
  float totalValue(const cv::Mat &hist);
  float maxValue(const cv::Mat &hist);
};

#endif // __FEATURES_HPP__
