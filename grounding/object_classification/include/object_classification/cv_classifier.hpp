
#ifndef __CV_CLASSIFIER_HPP__
#define __CV_CLASSIFIER_HPP__

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using std::string;

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;

class Classifier {
public:
  Classifier(const string& model_file,
	     const string& config_file,
	     const string& label_file);

  std::vector<Prediction> Classify(const cv::Mat& img, int N = -1);

private:
  std::shared_ptr<dnn::Net> net_;
  cv::Size input_geometry_;
  cv::Scalar input_mean_;
  std::vector<string> labels_;
};

Classifier::Classifier(const string& model_file,
                       const string& config_file,
                       const string& label_file) {

  /* Load the network. */
  net_ = std::make_shared<dnn::Net>(dnn::readNetFromCaffe(model_file, config_file));

  /* Set back-end: 
       DNN_BACKEND_DEFAULT          : automatically (by default), 
       DNN_BACKEND_HALIDE           : Halide language (http://halide-lang.org/), 
       DNN_BACKEND_INFERENCE_ENGINE : Intel's Deep Learning Inference Engine (https://software.intel.com/openvino-toolkit), 
       DNN_BACKEND_OPENCV           : OpenCV implementation 
  */
  net_->setPreferableBackend(dnn::DNN_BACKEND_DEFAULT);

  /* Set target device:
       DNN_TARGET_CPU         : CPU target (by default),
       DNN_TARGET_OPENCL      : OpenCL, 
       DNN_TARGET_OPENCL_FP16 : OpenCL fp16 (half-float precision),
       DNN_TARGET_MYRIAD      : VPU 
  */
  net_->setPreferableTarget(dnn::DNN_TARGET_CPU);

  /* Load labels. */
  std::ifstream labels(label_file.c_str());
  string line;
  while (std::getline(labels, line))
    labels_.push_back(string(line));

  /* Set input layer paramters. */
  input_geometry_ = cv::Size(224, 224);
  input_mean_ = cv::Scalar(104,117,123);  
}

static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
    return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
    std::vector<std::pair<float, int> > pairs;
    for (size_t i = 0; i < v.size(); ++i)
        pairs.push_back(std::make_pair(v[i], i));
    std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

    std::vector<int> result;
    for (int i = 0; i < N; ++i)
        result.push_back(pairs[i].second);
    return result;
}

/* Return the top N predictions. */
std::vector<Prediction> Classifier::Classify(const cv::Mat& img, int N) {

  /* Creat a blob and pass the blob through the network. */
  Mat blob = dnn::blobFromImage( img, 1.0f, input_geometry_, input_mean_, true, false);
  net_->setInput(blob);
  Mat prob = net_->forward();


  /* Post-process the result. */
  prob = prob.reshape(1, 1);
  std::vector<float> output;
  output.assign( prob.begin<float>(), prob.end<float>());
    
  /* Default (N = -1), use all labels */
  if( N < 0 ) {
    N = (int)labels_.size();
  }
    
  std::vector<int> maxN = Argmax(output, N);
  std::vector<Prediction> predictions;
  for (int i = 0; i < N; ++i) {
    int idx = maxN[i];
    predictions.push_back(std::make_pair(labels_[idx], output[idx]));
  }

  return predictions;
}

#endif // __CV_CLASSIFIER_HPP__
