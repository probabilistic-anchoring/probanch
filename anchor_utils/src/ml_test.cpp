
#include <anchor_utils/ml.hpp>

using namespace ml;

// accuracy
float evaluate(cv::Mat& predicted, cv::Mat& actual) {
  assert(predicted.rows == actual.rows);
  int t = 0;
  int f = 0;
  for(int i = 0; i < actual.rows; i++) {
    float p = predicted.at<float>(i,0);
    float a = actual.at<float>(i,0);
    if((p >= 0.0 && a >= 0.0) || (p <= 0.0 &&  a <= 0.0)) {
      t++;
    } else {
      f++;
    }
  }
  return (t * 1.0) / (t + f);
}

// --[ Main fn ]--- 
int main(int, char**) { 

  // Read all samples from DB
  cv::Mat data, labels;
  ml::read( "anchorexp", "dataset", data, labels);
  std::cout<< "Total samples: " << data.rows << "x" << data.cols << std::endl;
  std::cout<< "Total Llabels: " << labels.rows << "x" << labels.cols << std::endl;
  std::cout<< "---" << std::endl;

  // Split the data into traning and test data
  cv::Mat trainData, trainLabels, testData, testLabels;
  ml::split( data, labels, trainData, trainLabels, testData, testLabels);
  std::cout<< "Traning samples: " << trainLabels.rows << std::endl;
  std::cout<< "Test samples: " << testData.rows << std::endl;
  std::cout<< "---" << std::endl;

  // Filter the data
  cv::Mat trainFiltered, testFiltered;
  ml::filter( trainData, trainFiltered, 4);
  ml::filter( testData, testFiltered, 4);
  /*
  std::cout<< "Train data filtered: " << trainFiltered.rows << "x" << trainFiltered.cols << std::endl;
  std::cout<< "Test data filtered: " << testFiltered.rows << "x" << testFiltered.cols << std::endl;
  std::cout<< "---" << std::endl;
  */

  // Create and train a SVM classifier
  ml::MachinePtr classifier = ml::create("svm");
  classifier->train(trainData, trainLabels);

  // Test the classifier
  cv::Mat predicted(testData.rows, 1, CV_32F);
  for( uint i = 0; i < testData.rows; i++) {
    cv::Mat sample = testData.row(i);
    predicted.at<float>(i, 0) = classifier->predict(sample);
  }
  std::cout << "Accuracy_{SVM} = " << evaluate(predicted, testLabels) << std::endl;

}
