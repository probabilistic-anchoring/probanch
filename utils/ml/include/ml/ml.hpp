#ifndef __ML_HPP__
#define __ML_HPP__

#include <iostream>
#include <cmath>
#include <string>
#include <memory>
#include <map>

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
    Ptr<ml::StatModel> _model;
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
    ML(string type, Ptr<ml::StatModel> model) : _type(type), _model(model) {}
#endif
    ~ML() {}

    // --[ Traning/prediction ]--
    void train( const Mat &data, const Mat &labels); 
    float predict(const Mat &sample); 
    
    // --[ Save function]--
    void save(const string db_name, const string collection);
    void save(const string filename);
    
  }; // ...end of class. ]---

  // ---[ Type defines ]--- 
  // Typedefine a shared object pointer
  typedef std::shared_ptr<ML> MachinePtr; 


  // ---[ Statistics struct....
  struct Stats {

    // Used to store a "confusion matrix" 
    struct ConfusionMatrix {
      int _TP, _FP, _TN, _FN;
      ConfusionMatrix() : _TP(0), _FP(0), _TN(0), _FN(0) {}
      int getTotal() { return _TP + _FP + _TN + _FN; }
      float accuracy() { return (_TP + _TN) / (float)this->getTotal(); }
      float balancedAccuracy() { return (_TP / (float)(_TP + _FP) + _TN / (float)(_TN + _FN)) / 2.0; }
      float precision() { return _TP / (float)(_TP + _FP); }
      float recall() { return _TP / (float)(_TP + _FN); }
      float f1() { return 2.0 / (1.0 / this->recall() + 1.0 / this->precision()); }
      float diagnosticOddsRatio() { return (_TP / (float)_FP) / (_FN / (float)_TN); }
    };
    map<int, ConfusionMatrix> _stats;

    // Modifiers
    void addMatrix(int idx) {
      this->_stats.insert( pair<int, ConfusionMatrix>(idx, ConfusionMatrix()) );
    }
    void addTP(int idx, int value) { this->_stats[idx]._TP += value; }
    void addFP(int idx, int value) { this->_stats[idx]._FP += value; }
    void addTN(int idx, int value) { this->_stats[idx]._TN += value; }
    void addFN(int idx, int value) { this->_stats[idx]._FN += value; }
    
    // Getters
    float getAccuracy(int idx) { return this->_stats[idx].accuracy(); }
    float avgAccuracy() {
      float avg = 0.0;
      for( auto &stat : this->_stats ) {
	avg += stat.second.accuracy();
      }
      return avg / (float)this->_stats.size();
    }
    float stdAccuracy() {
      float std = 0.0;
      float avg = this->avgAccuracy();
      for( auto &stat : this->_stats ) {
	std += (stat.second.accuracy() - avg) * (stat.second.accuracy() - avg);
      }
      return std::sqrt(std / (float)this->_stats.size());
    }
    float getBalancedAccuracy(int idx) { return this->_stats[idx].balancedAccuracy(); }
    float avgBalancedAccuracy() {
      float avg = 0.0;
      for( auto &stat : this->_stats ) {
	avg += stat.second.balancedAccuracy();
      }
      return avg / (float)this->_stats.size();
    }
    float getF1(int idx) { return this->_stats[idx].f1(); }
    float avgF1() {
      float avg = 0.0;
      for( auto &stat : this->_stats ) {
	avg += stat.second.f1();
      }
      return avg / (float)this->_stats.size();
    }

  }; // ...end of class. ]---

  
  // ---[ Namespace functions ]---
  MachinePtr create(const string type);
  MachinePtr load(const string db_name, const string collection, const string type);
  MachinePtr load(const string filename, const string type);
  void read(string db_name, string collection, Mat &data, Mat &labels);
  void filter(const Mat &data, Mat &filtered, int idx);
  void split( const Mat &data, const Mat &labels,
	      Mat &trainData, Mat &trainLabels,
	      Mat &testData, Mat &testLabels,
	      float k = 0.7 );
  void distribute(const Mat &data, const Mat &labels,
		  Mat &distData, Mat &distLabels, int k = 800);
  void write2file(const Mat &data, const Mat &labels,
		  std::string f_name, std::string path = "");

} // namespace 'machine'

#endif // __ML_HPP__
