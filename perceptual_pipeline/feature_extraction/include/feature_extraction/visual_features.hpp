#ifndef __FEATURES_HPP__
#define __FEATURES_HPP__

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Features {
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
  Features( int numKeyPoints = 2500,
	    FeatureDetectorType detectorType = CV_BRISK );
  ~Features();

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

  // Color detection
  uint16_t keyPointColors( const cv::Mat &img, 
			   const std::vector<cv::KeyPoint> &points,
			   std::vector<uint16_t> &colors,
			   int kernelSize = 30,
			   int n = 15 );
  uint16_t maskColors( const cv::Mat &img,
		       const cv::Mat &mask,
		       std::vector<uint16_t> &colors);

  // For later feature to text mapping
  std::string getDetectorStr() { return this->_detectorStr; }
  int getDescriptorLevel() { return this->_descriptorLevel; }
  std::string colorMapping(uchar color);
  uchar colorMapping(const std::string color);  

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
  static cv::Mat equalizeIntensity(const cv::Mat& img);

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


  // Private helper functions
  uchar calcColor( const cv::Mat &image, const cv::Rect &region, int n);
  uchar colorIdx(uchar H, uchar S, uchar V);

}; 

#endif // __FEATURES_HPP__
