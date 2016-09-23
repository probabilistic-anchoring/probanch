/* ------------------------------------------ 


   ------------------------------------------ */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <feature_extraction/visual_features.hpp>

// Constructor
Features::Features( int numKeyPoints,
		    FeatureDetectorType detectorType ) : _numKeyPoints(numKeyPoints) {

  // Initialize random seed
  srand ( time(NULL) );

  // Set detector to use for database
  if( detectorType == CV_BRIEF ) {
    this->_detectorStr = "brief";
    this->_descriptorLevel = 32;
  }
  else if( detectorType == CV_ORB )  {
    this->_detectorStr = "orb";
    this->_descriptorLevel = 32;
  }
  else if( detectorType == CV_BRISK ) {
    this->_detectorStr = "brisk";
    this->_descriptorLevel = 64;
  }
  else {
    this->_detectorStr = "freak";
    this->_descriptorLevel = 64;
  }
  this->_detector = new cv::OrbFeatureDetector(this->_numKeyPoints);

  // Descriptor extractor
  if( detectorType == CV_BRIEF ) {
    this->_extractor = new cv::BriefDescriptorExtractor(this->_descriptorLevel);
  } 
  else if( detectorType == CV_ORB ) { 
    this->_extractor = new cv::ORB();  
  }
  else if( detectorType == CV_BRISK ) { 
    this->_extractor = new cv::BRISK();  
  }
  else {
    this->_extractor = new cv::FREAK();  
  }

  // Matcher (Hamming distance for binary descriptors).
  this->_matcher = new cv::BFMatcher(cv::NORM_HAMMING);
}

// Destructor - clean up...
Features::~Features() {
  this->_matcher.release();
  this->_extractor.release();
  this->_detector.release();
}

// Detect keypoints in a gray scale image
void Features::detect( const cv::Mat &img, 
		       std::vector<cv::KeyPoint> &keypoints) {
  this->_detector->detect( img, keypoints);

  // Filter keypoints - save 500 strongest
  //this->filterResponseN( keypoints, 500);
}

// Extract a descriptor from a set of keypoints
cv::Mat Features::extract( const cv::Mat &img, 
			   std::vector<cv::KeyPoint> &keypoints,
			   int n ) {
  cv::Mat descriptor;
  this->_extractor->compute( img, keypoints, descriptor);
  descriptor = this->filterResponseN( keypoints, descriptor, n);
  return descriptor;
}

// Match two descriptors
void Features::match( const cv::Mat &queryDescriptor, 
		      const cv::Mat &trainDescriptor,
		      std::vector<cv::DMatch> &matches,
		      bool symmetrical ) {
  if( symmetrical ) {
    std::vector<cv::DMatch> query_matches, train_matches;
    std::vector<std::vector<cv::DMatch> > knnMatches;
    this->_matcher->knnMatch( queryDescriptor, trainDescriptor, knnMatches, 2);
    this->filterMatches( knnMatches, query_matches);
    knnMatches.clear();
    this->_matcher->knnMatch( trainDescriptor, queryDescriptor, knnMatches, 2);
    this->filterMatches( knnMatches, train_matches);
    this->filterSymmetrical( query_matches, train_matches, matches);
  }
  else {
    std::vector<std::vector<cv::DMatch> > knnMatches;
    this->_matcher->knnMatch( queryDescriptor, trainDescriptor, knnMatches, 2);
    this->filterMatches( knnMatches, matches);
  }
}

// Match a descriptor against a set of descriptors
void Features::match( const cv::Mat &queryDescriptor, 
		      const std::vector<cv::Mat> &trainDescriptors,
		      std::vector<cv::DMatch> &matches) {
  // Add and train the matcher
  this->_matcher->add(trainDescriptors);
  this->_matcher->train();

  // Match and filter 
  std::vector<std::vector<cv::DMatch> > knnMatches;
  this->_matcher->knnMatch( queryDescriptor, knnMatches, 2);
  this->filterMatches( knnMatches, matches);

  // Clean up
  this->_matcher->clear();
}

// Color detection based on random color samples around each keypoint
uint16_t Features::keyPointColors( const cv::Mat &img, 
				   const std::vector<cv::KeyPoint> &points,
				   std::vector<uint16_t> &colors,
				   int kernelSize,
				   int n ) {
  cv::Mat imgHSV;
  cv::cvtColor( img, imgHSV, CV_BGR2HSV);
  
  // Initilize result arrays
  colors = std::vector<uint16_t>( 12, 0);  // 12 "primary" readable colors

  // Iterate all keypoints
  uint16_t num_samples = 0;
  uchar idx;
  std::vector<cv::KeyPoint>::const_iterator ite = points.begin();
  for( ; ite != points.end(); ++ite) {
    num_samples++;
    idx = this->calcColor( imgHSV, 
			   cv::Rect( ite->pt.x - kernelSize, 
				     ite->pt.y - kernelSize, 
				     kernelSize * 2, kernelSize * 2 ),
			   n );
    colors[(int)idx]++;     
  }
  return num_samples;
}

// Color histogram over the a masked part of the image 
uint16_t Features::maskColors( const cv::Mat &img,
			       const cv::Mat &mask, 
			       std::vector<uint16_t> &colors) {
  cv::Mat imgHSV; //, gray, mask;
  cv::cvtColor( img, imgHSV, CV_BGR2HSV);
  
  colors.clear();
  colors.resize(12,0);
  uint16_t num_samples = 0;

  for (int i=0; i<mask.rows; i++)  
    for (int j=0; j<mask.cols; j++)
      if (mask.at<uchar>(i,j) != 0) {
	num_samples++;
	uint idx = this->colorIdx( imgHSV.at<cv::Vec3b>(i,j)[0],
				   imgHSV.at<cv::Vec3b>(i,j)[1],
				   imgHSV.at<cv::Vec3b>(i,j)[2] );
	colors[(int)idx]++;
      }
  return num_samples;
}

// Mapping functions for converting between text and "color" indices
std::string Features::colorMapping(uchar color) {
  switch( color ) {
  case 1: return "gray";
  case 2: return "white";
  case 3: return "red";
  case 4: return "brown";
  case 5: return "orange";
  case 6: return "yellow";
  case 7: return "green";
  case 8: return "cyan";
  case 9: return "blue";
  case 10: return "purple";
  case 11: return "pink";
  default: return "black";  // Default
  }
}
uchar Features::colorMapping(const std::string color) {
  if( color == "gray" ) { return 1; }
  else if( color == "white" ) { return 2; }
  else if( color == "red" ) { return 3; }
  else if( color == "brown" ) { return 4; }
  else if( color == "orange" ) { return 5; }
  else if( color == "yellow" ) { return 6; }
  else if( color == "green" ) { return 7; }
  else if( color == "cyan" ) { return 8; }
  else if( color == "blue" ) { return 9; }
  else if( color == "purple" ) { return 10; }
  else if( color == "pink" ) { return 11; }
  return 0;
}

// Helper function for down sample the number of features
cv::Mat Features::filterResponseN( std::vector<cv::KeyPoint> &keypoints,
				   cv::Mat &descriptor,
				   int n) {  
  float best;
  int idx, ite = 0;
  std::vector<bool> keepers( keypoints.size(), false);
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
  cv::Mat result( 0, descriptor.cols, CV_8U);
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

// Helper function for filtering matches by ratio test
void Features::filterMatches( std::vector<std::vector<cv::DMatch> > &matches,
			      std::vector<cv::DMatch> &filtered,
			      bool bestMatches,
			      float ratio) {
  filtered = std::vector<cv::DMatch>();

  // For all matches
  std::vector<std::vector<cv::DMatch> >::iterator ite;
  for( ite = matches.begin(); ite != matches.end(); ++ite) {
    if( bestMatches ) {
      // If 2 NN has been identified
      if( ite->size() > 1 ) {
	// Check distance ratio...
	if( (*ite)[0].distance / (*ite)[1].distance < ratio) {
	  filtered.push_back( (*ite)[0] ); // Keeper...
	}
      } 
    }
    else {
      if( ite->size() > 1 ) {
	// Check distance ratio...
	if( (*ite)[0].distance / (*ite)[1].distance >= ratio) {
	  filtered.push_back( (*ite)[0] ); // Keeper...
	}
      } 
      else {
	filtered.push_back( (*ite)[0] ); // Keeper...
      }
    }
  }
}

// Remove matches that are not symmetrical 
void Features::filterSymmetrical( const std::vector<cv::DMatch> &matches1,
				  const std::vector<cv::DMatch> &matches2,
				  std::vector<cv::DMatch>& result ) {
  result = std::vector<cv::DMatch>();

  // For all matches in frame_1 -> frame_2
  std::vector<cv::DMatch>::const_iterator ite1, ite2;
  for( ite1 = matches1.begin(); ite1 != matches1.end(); ++ite1 ) {

    // For all matches frame_2 -> frame_1
    for( ite2 = matches2.begin(); ite2 != matches2.end(); ++ite2 ) {

      // Symmetrical test
      if( ite1->queryIdx == ite2->trainIdx &&
	  ite2->queryIdx == ite1->trainIdx ) {
	// Add symmetrical matches
	/*
	result.push_back( cv::DMatch( ite1->queryIdx, 
				      ite1->trainIdx, 
			 	      ite1->distance ) );
	*/
	result.push_back( (*ite1) );
	break;
      }
    }
  } 
  //std::cout<<" - Symmetrical: "<<result.size()<<std::endl;  
}

// Helper function for sub-divide a set of descriptors
cv::Mat Features::filterDescriptor( const cv::Mat& descriptor,
				    const std::vector<int>& index,
				    int type ) {
  cv::Mat filtered = cv::Mat( 0, descriptor.cols, type);
  for( uint i = 0; i < index.size(); i++) { 
    filtered.push_back(descriptor.row(index[i]));
    //    descriptor.row(index[i]).copyTo(filtered.row(i));
  }
  return filtered;
}

// Helper function for randomly sample the most dominent color
// ... within an image region
uchar Features::calcColor(const cv::Mat &image, const cv::Rect &region, int n) {
  int i, j, ite, idx;
    
  // Initilize color arrays
  std::vector<uint16_t> colors( 12, 0);  // 12 "primary" readable colors

  // Random samples 
  for( ite = 0; ite < n; ite++ ) {
    j = region.x + rand() % region.width;
    i = region.y + rand() % region.height;
    
    // Saftey check
    if( j < 0 ) j = 0;
    else if( j >= image.cols ) j = image.cols - 1;
    if( i < 0 ) i = 0;
    else if( i >= image.rows ) j = image.rows - 1;

    idx = this->colorIdx( image.at<cv::Vec3b>(i,j)[0],
			  image.at<cv::Vec3b>(i,j)[1],
			  image.at<cv::Vec3b>(i,j)[2] );
    colors[(int)idx]++;
  }

  // Find index of most frequent color
  idx = 0;
  int dominant = colors[idx];
  for( ite = 1; ite < (int)colors.size(); ite++) {
    if( colors[ite] > dominant ) {
      dominant = colors[ite];
      idx = ite;
    }
  }
  return (uchar)idx;
}


// Helper function for converting HSV color regions to a "color" indices
uchar Features::colorIdx(uchar H, uchar S, uchar V) {
  if( S <= 40 ) { // Black, gray or white -- S th. 85 in pre. v.
    if( V > 170 ) {
      return 2;  // White
    }
    else if( V > 85 ) {
      return 1; // Gray
    }
    return 0;  // Black
  }
  else {
    if( H >= 10 && H < 20 ) {  // Orange and brown
      if( V < 171 ) {
	return 4;  //  Brown
      }
      return 5;    // Orange
    }
    else if( H >= 20 && H < 36 ) { 
      return 6;     // Yellow
    } 
    else if( H >= 36 && H < 80 ) { 
      return 7;     // Green
    } 
    else if( H >= 80 && H < 96 ) { 
      return 8;     // Cyan
    } 
    else if( H >= 96 && H < 130 ) { 
      return 9;     // Blue
    } 
    else if( H >= 130 && H < 144 ) { 
      return 10;    // Purple
    } 
    else if( H >= 144 && H < 162 ) { 
      return 11;     // Pink
    } 
  }
  return 3;  // Red - defualt
}

  // Histogram equalization of RGB image 
cv::Mat Features::equalizeIntensity(const cv::Mat& img) {
    
  // Check the channels
  if( img.channels() >= 3 ) {
    cv::Mat ycrcb;
    cv::cvtColor( img, ycrcb, CV_BGR2YCrCb);
    
    std::vector<cv::Mat> channels;
    cv::split( ycrcb, channels);
    
    cv::equalizeHist(channels[0], channels[0]);
    
    cv::Mat result;
    cv::merge( channels, ycrcb);
    cv::cvtColor( ycrcb, result, CV_YCrCb2BGR);
      
    return result;
  }
  return img;
}
