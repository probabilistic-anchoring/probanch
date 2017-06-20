
#include <anchor_utils/ml.hpp>

using namespace ml;

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
}
