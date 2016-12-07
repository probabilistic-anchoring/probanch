/* ------------------------------------------ 


   ------------------------------------------ */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <feature_extraction/visual_features.hpp>

using namespace cv;
using namespace std;

/* ----------------------------

   Keypoint features

   --------------------------- */

// Constructor
KeypointFeatures::KeypointFeatures( int numKeyPoints,
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
  this->_detector = new OrbFeatureDetector(this->_numKeyPoints);

  // Descriptor extractor
  if( detectorType == CV_BRIEF ) {
    this->_extractor = new BriefDescriptorExtractor(this->_descriptorLevel);
  } 
  else if( detectorType == CV_ORB ) { 
    this->_extractor = new ORB();  
  }
  else if( detectorType == CV_BRISK ) { 
    this->_extractor = new BRISK();  
  }
  else {
    this->_extractor = new cv::FREAK();  
  }

  // Matcher (Hamming distance for binary descriptors).
  this->_matcher = new BFMatcher(NORM_HAMMING);
}

// Destructor - clean up...
KeypointFeatures::~KeypointFeatures() {
  this->_matcher.release();
  this->_extractor.release();
  this->_detector.release();
}

// Detect keypoints in a gray scale image
void KeypointFeatures::detect( const Mat &img, 
			       vector<KeyPoint> &keypoints) {
  this->_detector->detect( img, keypoints);

  // Filter keypoints - save 500 strongest
  //this->filterResponseN( keypoints, 500);
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

  // Clean up
  this->_matcher->clear();
}

// Helper function for down sample the number of features
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

// Helper function for filtering matches by ratio test
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
cv::Mat KeypointFeatures::filterDescriptor( const Mat& descriptor,
					    const vector<int>& index,
					    int type ) {
  Mat filtered = Mat( 0, descriptor.cols, type);
  for( uint i = 0; i < index.size(); i++) { 
    filtered.push_back(descriptor.row(index[i]));
  }
  return filtered;
}


/* ----------------------------

   Color features

   --------------------------- */

// Constructor
ColorFeatures::ColorFeatures(int hbins,
			     int sbins,
			     int vbins ) : _hbins(hbins), _sbins(sbins), _vbins(vbins) {

  // Classifier training data
  Mat labelsMat(0, 1, CV_32FC1);
  Mat dataMat(0, 1, CV_8UC3);
  buildTrainingData( white_shades, color_shade_numbers[SHADE_OF_WHITE], (float)SHADE_OF_WHITE, dataMat, labelsMat);
  buildTrainingData( gray_shades, color_shade_numbers[SHADE_OF_GRAY], (float)SHADE_OF_GRAY, dataMat, labelsMat);
  buildTrainingData( black_shades, color_shade_numbers[SHADE_OF_BLACK], (float)SHADE_OF_BLACK, dataMat, labelsMat);
  buildTrainingData( magenta_shades, color_shade_numbers[SHADE_OF_MAGENTA], (float)SHADE_OF_MAGENTA, dataMat, labelsMat);
  buildTrainingData( pink_shades, color_shade_numbers[SHADE_OF_PINK], (float)SHADE_OF_PINK, dataMat, labelsMat);
  buildTrainingData( red_shades, color_shade_numbers[SHADE_OF_RED], (float)SHADE_OF_RED, dataMat, labelsMat);
  buildTrainingData( brown_shades, color_shade_numbers[SHADE_OF_BROWN], (float)SHADE_OF_BROWN, dataMat, labelsMat);
  buildTrainingData( orange_shades, color_shade_numbers[SHADE_OF_ORANGE], (float)SHADE_OF_ORANGE, dataMat, labelsMat);
  buildTrainingData( yellow_shades, color_shade_numbers[SHADE_OF_YELLOW], (float)SHADE_OF_YELLOW, dataMat, labelsMat);
  buildTrainingData( green_shades, color_shade_numbers[SHADE_OF_GREEN], (float)SHADE_OF_GREEN, dataMat, labelsMat);
  buildTrainingData( cyan_shades, color_shade_numbers[SHADE_OF_CYAN], (float)SHADE_OF_CYAN, dataMat, labelsMat);
  buildTrainingData( blue_shades, color_shade_numbers[SHADE_OF_BLUE], (float)SHADE_OF_BLUE, dataMat, labelsMat);
  buildTrainingData( violet_shades, color_shade_numbers[SHADE_OF_VIOLET], (float)SHADE_OF_VIOLET, dataMat, labelsMat);


  // Convert training data to HSV color space
  Mat trainingDataMat(0, 3, CV_32FC1); 
  cvtColor( dataMat, dataMat, CV_BGR2HSV);
  for( int i = 0; i < dataMat.rows; i++ ) {
    Mat row(1, 3, CV_32FC1);
    row.at<float>(0,0) = dataMat.at<Vec3b>(i,0)[0];
    row.at<float>(0,1) = dataMat.at<Vec3b>(i,0)[1];
    row.at<float>(0,2) = dataMat.at<Vec3b>(i,0)[2];
    trainingDataMat.push_back(row);
  }

  // Set up SVM's parameters
  CvSVMParams params;
  params.svm_type    = CvSVM::C_SVC; //CvSVM::NU_SVC;
  params.kernel_type = CvSVM::LINEAR;
  params.nu = 0.1; // 0.1
  params.C = 0.01;
  params.term_crit = cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, (int)1e10, 1e-8); // 1e-6

  // Train the SVM
  this->_svm.train(trainingDataMat, labelsMat, Mat(), Mat(), params);
}

// Destructor - clean up...
ColorFeatures::~ColorFeatures() {
  //this->_svm.release();
}

// Color detection
void ColorFeatures::calculate( const Mat &img,
			       Mat &hist,
			       const Mat &mask ) {
  // Convert the image to HSV color space
  Mat hsv;
  cvtColor( img, hsv, CV_BGR2HSV);    

  // Quantize value levels and ranges
  int histSize[] = { _hbins, _sbins, _vbins};
  float hranges[] = { 0, (float)_hbins };
  float sranges[] = { 0, (float)_sbins };
  float vranges[] = { 0, (float)_vbins };
  const float* ranges[] = { hranges, sranges, vranges };

  // Compute the histogram for all channels
  int channels[] = {0, 1, 2};

  // Calculate the color histogram
  if( !mask.data ) {
    calcHist( &hsv, 1, channels, Mat(), // ...do not use mask
	      hist, 3, histSize, ranges,
	      true, // ...the histogram is uniform
	      false );
  }
  else {
    calcHist( &hsv, 1, channels, mask, 
	      hist, 3, histSize, ranges,
	      true, // ...the histogram is uniform
	      false );
  }
}

void ColorFeatures::predict( const Mat &hist,
			     vector<float> &preds) {
  // Initialize the result vector
  preds = vector<float>( COLOR_SHADES_MAX, 0.0);

  // Calculate the total sum of all histogram bins
  float totVal = this->totalValue(hist);

  // Predict the color distribution
  for( int h = 0; h < this->_hbins; h++ )
    for( int s = 0; s < this->_sbins; s++ ) 
      for( int v = 0; v < this->_vbins; v++ ) 
	if( hist.at<float>(h, s, v) > 0.0 ) {
	  Mat sampleMat = (Mat_<float>(1,3) << h,s,v);
	  float response = this->_svm.predict(sampleMat);
	  preds[(int)response] += hist.at<float>(h, s, v) / (float)totVal;
	}
}	


void ColorFeatures::normalize( Mat &hist ) {

  // Calculate the max value
  float maxVal = this->maxValue(hist);
  for( int h = 0; h < hist.rows; h++ )
    for( int s = 0; s < hist.cols; s++ )
      for( int v = 0; v < hist.channels(); v++ ) 
	hist.at<float>(h, s, v) /= maxVal;
}

void ColorFeatures::reduce( const Mat &hist,
			    Mat &result, 
			    int hbins, int sbins, int vbins ) {
  
  // Reduce the number of bins of the histogram
  int sz[] = { hbins, sbins, vbins};
  result = cv::Mat( 3, sz, CV_32FC1, Scalar::all(0.0));
  int h_idx = 0;
  for( int h = 0; h < this->_hbins; h++ ) {
    int s_idx = 0;
    for( int s = 0; s < this->_sbins; s++ ) { 
      int v_idx = 0;
      for( int v = 0; v < this->_vbins; v++ ) { 
	result.at<float>( h_idx, s_idx, v_idx) += hist.at<float>(h, s, v);
	if( v % (this->_vbins / vbins) == 0 ) { v_idx++; }
      }
      if( s % (this->_sbins / sbins) == 0 ) { s_idx++; }
    }
    if( h % (this->_hbins / hbins) == 0 ) { h_idx++; }
  }
}

// Mapping functions for converting between index and "color" symbols
string ColorFeatures::colorSymbol(int idx) {
  return color_shade_names[idx];
}

// Histogram equalization of RGB image 
cv::Mat ColorFeatures::equalizeIntensity(const Mat& img) {
    
  // Check the channels
  if( img.channels() >= 3 ) {
    Mat ycrcb;
    cvtColor( img, ycrcb, CV_BGR2YCrCb);
    
    vector<Mat> channels;
    split( ycrcb, channels);
    
    equalizeHist(channels[0], channels[0]);
    
    Mat result;
    merge( channels, ycrcb);
    cvtColor( ycrcb, result, CV_YCrCb2BGR);
      
    return result;
  }
  return img;
}

// Helper function for building the set of training data from examples
void ColorFeatures::buildTrainingData( const Color index[], 
				       int n,
				       float cl,
				       Mat &trainData,
				       Mat &labelData ) {
  for( int i = 0; i < n; i++ ) {
    Mat row(1, 1, CV_8UC3);
    row.at<Vec3b>(0,0)[0] = color_data[index[i]].rgb.b;
    row.at<Vec3b>(0,0)[1] = color_data[index[i]].rgb.g;
    row.at<Vec3b>(0,0)[2] = color_data[index[i]].rgb.r;
    trainData.push_back(row);
    labelData.push_back( cv::Mat( 1, 1, CV_32FC1, cl) );
  }
}

// Helper functions for for calculating the max and total value of a histogram
float ColorFeatures::totalValue(const Mat &hist) {
  float totVal = 0.0;
  for( int h = 0; h < hist.rows; h++ )
    for( int s = 0; s < hist.cols; s++ )
      for( int v = 0; v < hist.channels(); v++ ) 
	totVal += hist.at<float>(h, s, v);
  return totVal;
}
float ColorFeatures::maxValue(const Mat &hist) {

  float maxVal = 0.0;
  for( int h = 0; h < hist.rows; h++ )
    for( int s = 0; s < hist.cols; s++ )
      for( int v = 0; v < hist.channels(); v++ )
	if( hist.at<float>(h, s, v) > maxVal ) {
	  maxVal = hist.at<float>(h, s, v);
	}
  return maxVal;
}

