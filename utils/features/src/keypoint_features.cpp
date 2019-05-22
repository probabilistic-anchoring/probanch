/* ------------------------------------------ 
   keypoint_features.cpp
   ----------------------

   Class for handling of visual keypoint features.
 
   Modified on: May 21, 2019
   Author: Andreas Persson
   ------------------------------------------ */

#include <assert.h>
#include <boost/algorithm/string.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/xfeatures2d.hpp"
#endif
#include <features/visual.hpp>

// ---[ Used namespaces ]--- '
using namespace cv;
using namespace std;

namespace visual_2d {
  

  // --[ Constructor ] ---
  KeypointFeatures::KeypointFeatures( const string &detectorType,
				      int nfeatures ) {

    // Check the detector type
    assert( detectorType == "brief" ||
	    detectorType == "brisk"  ||
	    detectorType == "freak" ||
	    detectorType == "orb"   );
#if CV_MAJOR_VERSION == 3
    assert( detectorType == "daisy" );
#endif
    _detectorType = detectorType;
    
    // Feature detector
#if CV_MAJOR_VERSION == 2
    this->_detector = cv::ORB::ORB(nfeatures);
#elif CV_MAJOR_VERSION == 3
    this->_detector = cv::ORB::create(nfeatures);
#endif
    

  // Descriptor extractor
#if CV_MAJOR_VERSION == 2
    this->_extractor = cv::DescriptorExtractor::create(boost::to_upper_copy(_detectorType));
#elif CV_MAJOR_VERSION == 3
    if     ( detectorType == "brisk" ) this->_extractor = cv::BRISK::create();
    else if( detectorType == "orb"   ) this->_extractor = cv::ORB::create();
  #ifdef HAVE_OPENCV_XFEATURES2D
    else if( detectorType == "brief" ) cv::xfeatures2d::BriefDescriptorExtractor::create();
    else if( detectorType == "freak" ) cv::xfeatures2d::FREAK::create();
    else if( detectorType == "daisy" ) cv::xfeatures2d::DAISY::create();
  #else
    std::cout << "Descriptor type: '" << detectorType << "' requires xfeatures2d contrib module to be installed." << std::endl;
  #endif
#endif
  

    // Matcher (Hamming distance for binary descriptors).
    //this->_matcher = cv::BFMatcher::BFMatcher(NORM_HAMMING);
    this->_matcher = cv::DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_HAMMING);
  }

  // Detect keypoints in a gray scale image
  void KeypointFeatures::detect( const Mat &img, 
				 vector<KeyPoint> &keypoints,
				 const Mat &mask ) {
    if( !mask.data ) {
      this->_detector->detect( img, keypoints);
    }
    else {
      this->_detector->detect( img, keypoints, mask);
    }
  }
  
  // Extract a descriptor from a set of keypoints
  Mat KeypointFeatures::extract( const Mat &img, 
				 vector<KeyPoint> &keypoints,
				 int n ) {
    Mat descriptor;
    this->_extractor->compute( img, keypoints, descriptor);
    descriptor = this->filterResponseN( keypoints, descriptor, n);
    return descriptor;
  }

  // Match two descriptors
  void KeypointFeatures::match( const Mat &queryDescriptor, 
				const Mat &trainDescriptor,
				vector<DMatch> &matches,
				bool symmetrical ) {
    if( symmetrical ) {
      vector<DMatch> query_matches, train_matches;
      vector<vector<DMatch> > knnMatches;
      this->_matcher->knnMatch( queryDescriptor, trainDescriptor, knnMatches, 2);
      this->filterMatches( knnMatches, query_matches);
      knnMatches.clear();
      this->_matcher->knnMatch( trainDescriptor, queryDescriptor, knnMatches, 2);
      this->filterMatches( knnMatches, train_matches);
      this->filterSymmetrical( query_matches, train_matches, matches);
    }
    else {
      vector<vector<DMatch> > knnMatches;
      this->_matcher->knnMatch( queryDescriptor, trainDescriptor, knnMatches, 2);
      this->filterMatches( knnMatches, matches);
    }
  }
  
  // Match a descriptor against a set of descriptors
  void KeypointFeatures::match( const Mat &queryDescriptor, 
				const vector<Mat> &trainDescriptors,
				vector<DMatch> &matches) {
    // Add and train the matcher
    this->_matcher->add(trainDescriptors);
    this->_matcher->train();

    // Match and filter 
    vector<vector<DMatch> > knnMatches;
    this->_matcher->knnMatch( queryDescriptor, knnMatches, 2);
    this->filterMatches( knnMatches, matches);
    
    // Clean up...
    this->_matcher->clear();
  }



  // --[ Helper function ]----
  
  // Down sample the number of features
  cv::Mat KeypointFeatures::filterResponseN( vector<KeyPoint> &keypoints,
					     Mat &descriptor,
					     int n) {  
    float best;
    int idx, ite = 0;
    vector<bool> keepers( keypoints.size(), false);
    while( ite < n && ite < (int)keypoints.size() ) {
      best = 0.0;
      idx = -1;
      for( size_t i = 0; i < keypoints.size(); i++) {
	if( !keepers[i] && keypoints[i].response > best ) {
	  best = keypoints[i].response;
	  idx = i;
	}
      }
      if( idx >= 0 ) {
	keepers[idx] = true;
      }
      ite++;
    }
  
    // Remove a row from descriptor matrix (the bad way)
    Mat result( 0, descriptor.cols, CV_8U);
    for( int i = 0; i < descriptor.rows; i++) {
      if( keepers[i] ) {
	result.push_back(descriptor.row(i));
      }
    }
    for( int i = keepers.size() - 1; i >= 0; i--) {
      if( !keepers[i] ) {
	keypoints.erase( keypoints.begin() + i );
      }
    }
    return result;
  }

  // Filtering matches by ratio test
  void KeypointFeatures::filterMatches( vector<vector<DMatch> > &matches,
					vector<DMatch> &filtered,
					bool bestMatches,
					float ratio) {
    filtered = vector<DMatch>();

    // For all matches
    vector<vector<DMatch> >::iterator ite;
    for( ite = matches.begin(); ite != matches.end(); ++ite) {
      if( bestMatches ) {
	// If 2 NN has been identified
	if( ite->size() > 1 ) {
	  // Check distance ratio...
	  if( (*ite)[0].distance / (*ite)[1].distance < ratio) {
	    filtered.push_back( (*ite)[0] ); // ...keeper.
	  }
	} 
      }
      else {
	if( ite->size() > 1 ) {
	  // Check distance ratio...
	  if( (*ite)[0].distance / (*ite)[1].distance >= ratio) {
	    filtered.push_back( (*ite)[0] ); //  ...keeper.
	  }
	} 
	else {
	  filtered.push_back( (*ite)[0] ); //  ...keeper.
	}
      }
    }
  }

  // Remove matches that are not symmetrical 
  void KeypointFeatures::filterSymmetrical( const vector<DMatch> &matches1,
					    const vector<DMatch> &matches2,
					    vector<DMatch>& result ) {
    result = vector<DMatch>();

    // For all matches in frame_1 -> frame_2
    vector<DMatch>::const_iterator ite1, ite2;
    for( ite1 = matches1.begin(); ite1 != matches1.end(); ++ite1 ) {

      // For all matches frame_2 -> frame_1
      for( ite2 = matches2.begin(); ite2 != matches2.end(); ++ite2 ) {

	// Symmetrical test
	if( ite1->queryIdx == ite2->trainIdx &&
	    ite2->queryIdx == ite1->trainIdx ) {
	  result.push_back( (*ite1) ); // ...add symmetrical matches to the result.
	  break;
	}
      }
    } 
  }

  // Sub-divide a set of descriptors
  cv::Mat KeypointFeatures::filterDescriptor( const Mat& descriptor,
					      const vector<int>& index,
					      int type ) {
    Mat filtered = Mat( 0, descriptor.cols, type);
    for( uint i = 0; i < index.size(); i++) { 
      filtered.push_back(descriptor.row(index[i]));
    }
    return filtered;
  }

} // ...namespace ('visual_2d') ]---



