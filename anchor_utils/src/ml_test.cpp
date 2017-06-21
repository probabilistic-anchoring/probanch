
#include <anchor_utils/ml.hpp>

using namespace ml;

// --[ Main fn ]--- 
int main(int, char**) { 

  // Read all samples from DB
  cv::Mat data, labels;
  ml::read( "anchorexp", "dataset", data, labels);
  std::cout<< "Total samples: " << data.rows << "x" << data.cols << std::endl;
  std::cout<< "Total labels: " << labels.rows << "x" << labels.cols << std::endl;
  std::cout<< "---" << std::endl;

  // Split the data into traning and test data
  cv::Mat trainData, trainLabels, testData, testLabels;
  ml::split( data, labels, trainData, trainLabels, testData, testLabels, 0.7);
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

  // Create, train aand test SVM classifier
  ml::MachinePtr classifier = ml::create("svm");
  //classifier->train( trainFiltered, trainLabels);
  classifier->train( trainData, trainLabels);
  int correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    //cv::Mat sample = testFiltered.row(i);
    cv::Mat sample = testData.row(i);
    float pred = classifier->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{SVM} = " << correct / (float)testData.rows << std::endl;
  classifier->save("anchortmpdb", "ml");
  //classifier->save("anchorexp", "ml");

  // Train and test MLP classifier
  classifier = ml::create("mlp");
  //classifier->train( trainFiltered, trainLabels);
  classifier->train( trainData, trainLabels);
  correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    //cv::Mat sample = testFiltered.row(i);
    cv::Mat sample = testData.row(i);
    float pred = classifier->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{MLP} = " << correct / (float)testData.rows << std::endl;

  // Train and test kNN classifier
  classifier = ml::create("knn");
  //classifier->train( trainFiltered, trainLabels);
  classifier->train( trainData, trainLabels);
  correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    //cv::Mat sample = testFiltered.row(i);
    cv::Mat sample = testData.row(i);
    float pred = classifier->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{kNN} = " << correct / (float)testData.rows << std::endl;

  // Train and test Bayes classifier
  classifier = ml::create("bayes");
  //classifier->train( trainFiltered, trainLabels);
  classifier->train( trainData, trainLabels);
  correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    //cv::Mat sample = testFiltered.row(i);
    cv::Mat sample = testData.row(i);
    float pred = classifier->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{Bayes} = " << correct / (float)testData.rows << std::endl;

  // Train and test Decesion tree classifier
  classifier = ml::create("tree");
  //classifier->train( trainFiltered, trainLabels);
  classifier->train( trainData, trainLabels);
  correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    //cv::Mat sample = testFiltered.row(i);
    cv::Mat sample = testData.row(i);
    float pred = classifier->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{DTree} = " << correct / (float)testData.rows << std::endl;

  /*
  // Load train classifier
  ml::MachinePtr classifier2 = ml::load("anchorexp", "ml", "svm");
  correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    cv::Mat sample = testData.row(i);
    float pred = classifier2->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{SVM loaded} = " << correct / (float)testData.rows << std::endl;
  */
}
