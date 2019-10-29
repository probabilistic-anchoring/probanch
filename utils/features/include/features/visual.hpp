#ifndef __VISUAL_HPP__
#define __VISUAL_HPP__

#include <cmath> 
#include <iostream>
#include <vector>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ml/ml.hpp>

#include <features/shades.hpp>

namespace visual_2d {

  // ---[ Used namespaces ]---
  using namespace std;
  using namespace cv;
  
  // ---[ Class for color features
  // ----------------------------------------
  class ColorFeatures {

    // SVM classifier
    machine::MachinePtr _svm; 
  
    // Scale factor (in range: [0.0, 1.0]), and histogram bins.
    float _scale;
    int _hbins, _sbins, _vbins;
    
    // --[ Private helper function ]--
    void buildTrainingData( const Color index[], 
			    int n,
			    float cl,
			    Mat &trainData,
			    Mat &labelData );
    float totalValue(const Mat &hist);
    float maxValue(const Mat &hist);

  public: // ...public space...

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

    // Static helper functions
    static Mat equalizeIntensity(const Mat& img);

    // Getter/setter functions
    string getColorSymbol(int idx);
    Scalar getColorScalar(int pred);
    void setScaleFactor(float val) { if( val >= 0.0 && val <= 1.0 ) this->_scale = val; }
    float getScaleFactor() { return this->_scale; }
      
  }; // ...end of class. ]---


  // ---[ Class for color features
  // ----------------------------------------
  class KeypointFeatures {

    // Feature detector and descriptor extractor object
    Ptr<FeatureDetector> _detector;
    Ptr<DescriptorExtractor> _extractor;

    // Feature matcher object
    Ptr<DescriptorMatcher> _matcher;

    // Descriptor information
    string _detectorType;
    
  public: // ...public space...
    
    // --[ Constructor(s)/destructor ]-- 
    KeypointFeatures( const string &detectorType,
		      int nfeatures = 500 );
    ~KeypointFeatures() {};

    // Feature extracting and detecting
    void detect( const cv::Mat &img,
		 std::vector<cv::KeyPoint> &keypoints,
		 const Mat &mask = Mat() );
    Mat extract( const cv::Mat &img, 
		 std::vector<cv::KeyPoint> &keypoints,
		 int n = 500 );

    // Feature matching (not used at the monment)
    void match( const cv::Mat &queryDescriptor, 
		const cv::Mat &trainDescriptor,
		std::vector<cv::DMatch> &matches,
		bool symmetrical = false);
    void match( const cv::Mat &queryDescriptor, 
		const std::vector<cv::Mat> &trainDescriptors,
		std::vector<cv::DMatch> &matches);

    
    // Getters/setters
    std::string getDetectorType() { return this->_detectorType; }

    // --[ Static helper functions --]
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
    
  }; // ...end of class. ]---


} // namespace 'visual_2d'

#endif // __VISUAL_HPP__
