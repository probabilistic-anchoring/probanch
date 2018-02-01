#ifndef __ML_HPP__
#define __ML_HPP__

#include <iostream>
#include <cmath>
#include <string>
#include <memory>

// OpenCV includes
#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

namespace machine {

  // ---[ Used namespaces ]---
  using namespace std;
  using namespace cv;
  
  // ---[ Class definition...
  // --------------------------------
  class ML {
    string _type;
    Mat _var_type;

    // Support for both opencv2 and opencv3
#if CV_MAJOR_VERSION == 2
    shared_ptr<CvStatModel> _model;
    CvSVMParams _svm_p;
    CvANN_MLP_TrainParams _mlp_p;
#elif CV_MAJOR_VERSION == 3
    Ptr<Algorithm> _model;
#endif

  public:

    // --[ Constructor(s)/destructor --]
#if CV_MAJOR_VERSION == 2
    ML(string type, shared_ptr<CvStatModel> model) : _type(type), _model(model) {}
    ML(string type, shared_ptr<CvStatModel> model, CvSVMParams params ) 
      : _type(type), _model(model), _svm_p(params) {}
    ML(string type, shared_ptr<CvStatModel> model, CvANN_MLP_TrainParams params ) 
      : _type(type), _model(model), _mlp_p(params) {}
    ML(string type, shared_ptr<CvStatModel> model, Mat var_type ) 
      : _type(type), _model(model), _var_type(var_type) {}
#elif CV_MAJOR_VERSION == 3
    ML(string type, Ptr<Algorithm> model) : _type(type), _model(model) {}
#endif
    ~ML() {}

    // --[ Traning/prediction ]--
    void train( const Mat &data, const Mat &labels); 
    float predict(const Mat &sample); 
    
    // --[ Save function]--
    void save(string db_name, string collection);
    void save(string filename);
    
  }; // ...end of class. ]---

  // ---[ Type defines ]--- 
  // Typedefine a smart object pointer
  typedef std::shared_ptr<ML> MachinePtr; 


  // ---[ Namespace functions ]---
  MachinePtr create(string type);
  MachinePtr load(string db_name, string collection, string type);
  MachinePtr load(string filename);
  void read(string db_name, string collection, Mat &data, Mat &labels);
  void filter(const Mat &data, Mat &filtered, int idx);
  void split( const Mat &data, const Mat &labels,
	      Mat &trainData, Mat &trainLabels,
	      Mat &testData, Mat &testLabels,
	      float k = 0.7 );
  void distribute(const Mat &data, const Mat &labels,
		  Mat &distData, Mat &distLabels, int k = 800);

} // namespace 'machine'

#endif // __ML_HPP__
