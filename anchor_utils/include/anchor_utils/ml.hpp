#ifndef __ML_HPP__
#define __ML_HPP__

#include <iostream>
#include <cmath>
#include <string>
#include <memory>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
/*
#include <cv.h>
#include <ml.h>
#include <highgui.h>
*/

namespace ml {

  // ---[ Used namespaces ]---
  using namespace std;
  using namespace cv;
  
    // ---[ Class definition...
  // --------------------------------
  class ML {
    ML() {}
    ~ML() {}

  private:
    

  }; // ...end of class. ]---

  // ---[ Type defines ]--- 
  // Typedefine a smart object pointer
  typedef std::shared_ptr<ML> MachinePtr; 


  // ---[ Namespace functions ]---
  MachinePtr create(string type);
  void read(string db_name, string collection, Mat &data, Mat &labels);
  void filter(const Mat &data, Mat &filtered, int idx);
  void split( const Mat &data, const Mat &labels,
	      Mat &trainData, Mat &trainLabels,
	      Mat &testData, Mat &testLabels,
	      float k = 0.7 );


} // namespace 'ml'

#endif // __ML_HPP__
