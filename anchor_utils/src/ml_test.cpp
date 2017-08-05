
#include <anchor_utils/ml.hpp>

using namespace machine;

// --[ Main fn ]--- 
int main(int argc, char* argv[]) { 
  
  if( argc != 2 ) {
    std::cerr << "Usage: " << argv[0] << " iterations" << std::endl; 
    return 1;
  }
  int ite = atoi(argv[1]);
  std::cout << "Running " << ite << " iteratons... " << std::endl; 

  // Read all samples from DB
  cv::Mat data, labels;
  machine::read( "anchorexp2", "dataset", data, labels);
  std::cout<< "Total samples: " << data.rows << "x" << data.cols << std::endl;
  std::cout<< "Total labels: " << labels.rows << "x" << labels.cols << std::endl;
  std::cout<< "---" << std::endl;

  // Split the data into traning and test data
  cv::Mat trainData, trainLabels, testData, testLabels;
  machine::split( data, labels, trainData, trainLabels, testData, testLabels, 0.7);
  std::cout<< "Traning samples: " << trainLabels.rows << std::endl;
  std::cout<< "Test samples: " << testData.rows << std::endl;
  std::cout<< "---" << std::endl;

  // opencv2
#if CV_MAJOR_VERSION == 2

  // Run everything
  int best = 0;
  std::vector<int> result( 5, 0);
  for ( uint x = 0; x < ite; x++) { 

    /*
    // Filter the data
    cv::Mat trainFiltered, testFiltered;
    machine::filter( trainData, trainFiltered, 4);
    machine::filter( testData, testFiltered, 4);
    
    std::cout<< "Train data filtered: " << trainFiltered.rows << "x" << trainFiltered.cols << std::endl;
    std::cout<< "Test data filtered: " << testFiltered.rows << "x" << testFiltered.cols << std::endl;
    std::cout<< "---" << std::endl;
    */

    // Create, train and test SVM classifier
    machine::MachinePtr classifier = machine::create("svm");
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
    //std::cout << "Accuracy_{SVM} = " << correct / (float)testData.rows << std::endl;
    //classifier->save("anchortmpdb", "machine");
    //classifier->save("anchorexp", "machine");
    if( correct > best ) {
      classifier->save("svm.yml");
      best = correct;
      std::cout << "Best accuracy_{SVM} = " << correct / (float)testData.rows << std::endl;
    }
    result[0] += correct;

    // Train and test MACHINEP classifier
    classifier = machine::create("machinep");
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
    //std::cout << "Accuracy_{MACHINEP} = " << correct / (float)testData.rows << std::endl;
    //classifier->save("anchortmpdb", "machine");
    result[1] += correct;

    // Train and test kNN classifier
    classifier = machine::create("knn");
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
    //std::cout << "Accuracy_{kNN} = " << correct / (float)testData.rows << std::endl;
    result[2] += correct;

    // Train and test Bayes classifier
    classifier = machine::create("bayes");
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
    //std::cout << "Accuracy_{Bayes} = " << correct / (float)testData.rows << std::endl;
    result[3] += correct;
    
    // Train and test Decesion tree classifier
    classifier = machine::create("tree");
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
    //std::cout << "Accuracy_{DTree} = " << correct / (float)testData.rows << std::endl;
    result[4] += correct;

    // Randomachiney make new split of training/test data...
    machine::split( data, labels, trainData, trainLabels, testData, testLabels, 0.7);    
  }
  std::cout << "--------" << std::endl;
  std::cout << "Accuracy_{SVM} = " << result[0] / (float)(testData.rows * ite) << std::endl;
  std::cout << "Accuracy_{MACHINEP} = " << result[1] / (float)(testData.rows * ite) << std::endl;  
  std::cout << "Accuracy_{kNN} = " << result[2] / (float)(testData.rows * ite) << std::endl;  
  std::cout << "Accuracy_{Bayes} = " << result[3] / (float)(testData.rows * ite) << std::endl;  
  std::cout << "Accuracy_{Dtree} = " << result[4] / (float)(testData.rows * ite) << std::endl;

  /*
  // Load train classifier
  machine::MachinePtr classifier2 = machine::load("anchorexp", "machine", "svm");
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

  // opencv3
#elif CV_MAJOR_VERSION == 3

  // Create, train and test SVM classifier
  machine::MachinePtr classifier = machine::create("svm");
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
  classifier->save("svm.yml");

  // Load train classifier
  machine::MachinePtr classifier2 = machine::load("svm.yml");
  correct = 0;
  for( uint i = 0; i < testData.rows; i++) {
    cv::Mat sample = testData.row(i);
    float pred = classifier2->predict(sample);
    if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
      correct++;
    }
  }
  std::cout << "Accuracy_{SVM loaded} = " << correct / (float)testData.rows << std::endl;

#endif

}
