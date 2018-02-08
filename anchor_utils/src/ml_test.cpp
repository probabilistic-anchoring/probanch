
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

  // Randomly distribute the data
  cv::Mat distData, distLabels;
  machine::distribute( data, labels, distData, distLabels);
  
  // Split the data into traning and test data
  cv::Mat trainData, trainLabels, testData, testLabels;
  machine::split( distData, distLabels, trainData, trainLabels, testData, testLabels, 0.7);
  std::cout<< "Traning samples: " << trainLabels.rows << std::endl;
  std::cout<< "Test samples: " << testData.rows << std::endl;
  std::cout<< "---" << std::endl;

  // opencv2
#if CV_MAJOR_VERSION == 2

  // Run everything
  int best = 0;
  std::vector<int> result( 5, 0);
  for ( uint x = 0; x < ite; x++) { 

    // Filter the data
    /*
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
  std::cout << "Accuracy_{MLP} = " << result[1] / (float)(testData.rows * ite) << std::endl;  
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
  
  
    // Run everything
  int best = 0;
  std::vector<int> result( 13, 0);
  std::vector<int> positions( 5, 0);

  // Statisitcs
  std::vector<cv::Mat> summaries( 4, cv::Mat( 0, 5, CV_64F) );
  
  double total = 0.0;
  for ( uint x = 0; x < ite; x++) { 

    /*
    // Filter the data
    cv::Mat trainFiltered, testFiltered;
    machine::filter( trainFiltered, trainFiltered, 0);
    machine::filter( testFiltered, testFiltered, 0);
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
      /*
      if( pred > 0.5 ) {
	pred = 1.0;
      }
      else {
	pred = 0.0;
      }
      */
      if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
	correct++;
	if( (int)pred > 0.5 ) {
	  summaries[0].push_back(sample);
	}
	else {
	  summaries[2].push_back(sample);
	}
      }
      else {
	if( (int)pred > 0.5 ) {
	  summaries[1].push_back(sample);
	}
	else {
	  summaries[3].push_back(sample);
	}
      }

    }
    if( correct > best ) {
      classifier->save("svm.yml");
      best = correct;
      std::cout << "Best accuracy_{SVM} = " << correct / (float)testData.rows << std::endl;
    }
    result[0] += correct;
    //std::cout << "Accuracy_{SVM loaded} = " << correct / (float)testData.rows << std::endl;
    
    /*
    for( uint q = 0; q < result.size(); q++) {
    
      // Test threshold based
      int correct = 0;
      for( uint i = 0; i < testData.rows; i++) {
	//cv::Mat sample = testFiltered.row(i);
	cv::Mat sample = testData.row(i);
	//std::cout << sample.rows << " x " << sample.cols << std::endl; 
	double tmp, pred;
	Point minLoc, maxLoc;
	minMaxLoc( sample, &tmp, &pred, &minLoc, &maxLoc);
	positions[maxLoc.x]++;
	total += 1.0;
	if( pred >= (0.4 + q * 0.05) ) {
	  pred = 1.0;
	}
	else {
	  pred = 0.0;
	}
	if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
	  correct++;
	}
      }
      result[q] += correct;
    }
    */
    
    // Randomly make new split of training/test data...
    machine::distribute( data, labels, distData, distLabels);
    machine::split( distData, distLabels, trainData, trainLabels, testData, testLabels, 0.7);    
  }
  
  std::cout << "--------" << std::endl;
  std::cout << "Accuracy_{SVM} = " << result[0] / (float)(testData.rows * ite) << std::endl;
  //std::cout << "Accuracy_{Th} = " << result[1] / (float)(testData.rows * ite) << std::endl;
  /*
  std::cout << "Accuracy_{MACHINEP} = " << result[1] / (float)(testData.rows * ite) << std::endl;  
  std::cout << "Accuracy_{kNN} = " << result[2] / (float)(testData.rows * ite) << std::endl;  
  std::cout << "Accuracy_{Bayes} = " << result[3] / (float)(testData.rows * ite) << std::endl;  
  std::cout << "Accuracy_{Dtree} = " << result[4] / (float)(testData.rows * ite) << std::endl;
  
  std::cout << "--------" << std::endl;
  for( uint q = 0; q < result.size(); q++) {
    std::cout << "Accuracy_{Th: " << (0.4 + q * 0.05) << "}:" << result[q] / (float)(testData.rows * ite) << std::endl;
  }
  std::cout << "--------" << std::endl;
  std::cout << "Influence (CAFFE):" << positions[0] / total << std::endl;
  std::cout << "Influence (COLOR):" << positions[1] / total << std::endl;
  std::cout << "Influence (POSITION):" << positions[2] / total << std::endl;
  std::cout << "Influence (SHAPE):" << positions[3] / total << std::endl;
  std::cout << "Influence (TIME):" << positions[4] / total << std::endl;
  */

  cv::Scalar m, std;
  std::cout << "True positive: " << std::endl;
  for( uint i = 0; i < 5; i++) {
    cv::meanStdDev( summaries[0].col(i), m, std);
    std::cout << m.val[0] << " " << std.val[0] << std::endl;
  }
  std::cout << "False positive: " << std::endl;
  for( uint i = 0; i < 5; i++) {
    cv::meanStdDev( summaries[1].col(i), m, std);
    std::cout << m.val[0] << " " << std.val[0] << std::endl;
  }
  std::cout << "True negative: " << std::endl;
  for( uint i = 0; i < 5; i++) {
    cv::meanStdDev( summaries[2].col(i), m, std);
    std::cout << m.val[0] << " " << std.val[0] << std::endl;
  }
  std::cout << "False negative: " << std::endl;
  for( uint i = 0; i < 5; i++) {
    cv::meanStdDev( summaries[3].col(i), m, std);
    std::cout << m.val[0] << " " << std.val[0] << std::endl;
  }
  
#endif

}
