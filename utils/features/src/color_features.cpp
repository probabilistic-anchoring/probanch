/* ------------------------------------------ 
   color_features.cpp
   ----------------------

   Class for handling of color features.
 
   Modified: May 22, 2019
   Author: Andreas Persson
   ------------------------------------------ */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <features/visual.hpp>

// ---[ Used namespaces ]--- 
using namespace cv;
using namespace std;


// ---[ Within namespace...
namespace visual_2d {
  
  // Constructor(s)
  ColorFeatures::ColorFeatures(const std::string &filename) {
    this->_svm = machine::load(filename, "svm");
  }
  ColorFeatures::ColorFeatures(int hbins,
			       int sbins,
			       int vbins ) : _hbins(hbins), _sbins(sbins), _vbins(vbins), _scale(1.0) {}


  // Load a pre-trained model
  void ColorFeatures::load(const std::string &filename) {
    this->_svm = machine::load(filename, "svm");
  }

  // Calcualte a color histogram (HSV color space)
  void ColorFeatures::calculate( const Mat &img,
				 Mat &hist,
				 const Mat &mask ) {
    // Convert the image to HSV color space
    if( img.data ) {
      Mat hsv;
      cvtColor( img, hsv, CV_BGR2HSV);    

      // Quantize value levels and ranges
      int histSize[] = {  (int)(_hbins * _scale), (int)(_sbins * _scale), (int)(_vbins * _scale)};
      float hranges[] = { 0, (float)_hbins };
      float sranges[] = { 0, (float)_sbins };
      float vranges[] = { 0, (float)_vbins };
      const float* ranges[] = { hranges, sranges, vranges };
      
      // Compute the histogram for all channels
      int channels[] = {0, 1, 2};
      
      // Calculate the color histogram
      if( !mask.data ) {
	cv::calcHist( &hsv, 1, channels, Mat(), // ...do not use mask
		      hist, 3, histSize, ranges,
		      true, // ...the histogram is uniform
		      false );
      }
      else {
	cv::calcHist( &hsv, 1, channels, mask, 
		      hist, 3, histSize, ranges,
		      true, // ...the histogram is uniform
		      false );
      }
    }
  }

  // Predice a discretized color histogram
  void ColorFeatures::predict( const Mat &hist,
			       vector<float> &preds) {
    // Initialize the result vector
    preds = vector<float>( COLOR_SHADES_MAX, 0.0);

    // Calculate the total sum of all histogram bins
    float totVal = 0.0;

    // Predict the color distribution
    for( int h = 0; h < this->_hbins * _scale; h++ )
      for( int s = 0; s < this->_sbins * _scale; s++ ) 
	for( int v = 0; v < this->_vbins * _scale; v++ ) 
	  if( hist.at<float>(h, s, v) > 0.0 ) {
	    Mat sampleMat = (Mat_<float>(1,3) << (h / _scale), (s / _scale), (v / _scale));
	    float response = this->_svm->predict(sampleMat);
	    //preds[(int)response] += hist.at<float>(h, s, v) / (float)totVal;
	    float val = hist.at<float>( h, s, v);
	    preds[(int)response] += val;
	    totVal += val;
	  }
    preds[SHADES_OF_RED_LOW] += preds[SHADES_OF_RED_HIGH];
    preds[SHADES_OF_RED_HIGH] = 0.0;
    for( uint i = 0; i < preds.size(); i++) {
      preds[i] = preds[i] / totVal;   
    }
  }	
  
  // Normalize a color histogram
  void ColorFeatures::normalize( Mat &hist ) {

    // Calculate the max value
    float maxVal = this->maxValue(hist);
    for( int h = 0; h < this->_hbins; h++ )
      for( int s = 0; s < this->_sbins; s++ ) 
	for( int v = 0; v < this->_vbins; v++ ) 
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
  
  // Mapping functions for converting between index and 'color' symbols
  string ColorFeatures::getColorSymbol(int idx) {
    return color_shade_names[idx];
  }

  Scalar ColorFeatures::getColorScalar( int pred ) {
    Scalar c;
    switch(pred) {
    case 0: c = Scalar( 242, 242, 242); break; // white
    case 1: c = Scalar( 181, 181, 181); break; // gray
    case 2: c = Scalar( 27, 27, 27); break;    // black
    case 3: c = Scalar( 252, 15, 252); break;  // magenta
    case 4: c = Scalar( 253, 116, 252); break; // pink
    case 5: c = Scalar( 15, 15, 255); break;   // red
    case 6: c = Scalar( 15, 15, 255); break;   // red
    case 7: c = Scalar( 42, 42, 165); break;  // brown
    case 8: c = Scalar( 15, 127, 255); break;   // orange
    case 9: c = Scalar( 15, 255, 255); break;   // yellow
    case 10: c = Scalar( 15, 255, 15); break;   // green
    case 11: c = Scalar( 242, 242, 128); break; // cyan
    case 12: c = Scalar( 255, 15, 15); break;   // blue
    case 13: c = Scalar( 255, 15, 143); break;  // violet
    case 14: c = Scalar( 127, 15, 127); break;  // purple
    }
    return c;
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
    for( int h = 0; h < this->_hbins; h++ )
      for( int s = 0; s < this->_sbins; s++ ) 
	for( int v = 0; v < this->_vbins; v++ ) 
	  totVal += hist.at<float>(h, s, v);
    return totVal;
  }
  float ColorFeatures::maxValue(const Mat &hist) {
    
    float maxVal = 0.0;
    for( int h = 0; h < this->_hbins; h++ )
      for( int s = 0; s < this->_sbins; s++ ) 
	for( int v = 0; v < this->_vbins; v++ ) 
	  if( hist.at<float>(h, s, v) > maxVal ) {
	    maxVal = hist.at<float>(h, s, v);
	  }
    return maxVal;
  }

} // ...namespace ('visual_2d') ]---




