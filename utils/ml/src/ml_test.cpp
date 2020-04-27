
#include <ml/ml.hpp>

using namespace machine;

// --[ Main fn ]--- 
int main(int argc, char* argv[]) { 

  int exclude = -1;
  if( argc < 2 or argc > 3 ) {
    std::cerr << "Usage: " << argv[0] << " iterations [no_class/no_time]" << std::endl; 
    return 1;
  }
  if( argc == 3 ) {
    if( std::string(argv[2]) == "no_class" ) {
      exclude = 0;
    }
    else if( std::string(argv[2]) == "no_color" ) {
      exclude = 1;
    }
    else if( std::string(argv[2]) == "no_pos" ) {
      exclude = 2;
    }
    else if( std::string(argv[2]) == "no_size" ) {
      exclude = 3;
    }
    else if( std::string(argv[2]) == "no_time" ) {
      exclude = 4;
    }
  }
  int ite = atoi(argv[1]);
  std::cout << "Running " << ite << " iteratons... " << std::endl; 
  
  cv::Mat data, labels;

  // Read all samples from JSON file
  // machine::file2data( data, labels, "reground_dataset.json", "./");
  
  // Read all samples from DB
  machine::read( "anchorexpC", "dataset", data, labels);
  std::cout<< "Total samples: " << data.rows << "x" << data.cols << std::endl;
  std::cout<< "Total labels: " << labels.rows << "x" << labels.cols << std::endl;
  std::cout<< "---" << std::endl;

  // Filter the data / remove one attribute
  if( exclude >= 0 ) {
    machine::filter( data, data, exclude);
    std::cout<< "Filtered total samples: " << data.rows << "x" << data.cols << std::endl;
    std::cout<< "---" << std::endl;
  }

  // Write data to JSON file
  machine::write2file( data, labels, "reground_dataset.json", "./");
  
  // Randomly distribute the data
  // cv::Mat distData, distLabels;
  // machine::distribute( data, labels, distData, distLabels, 100);
  
  // Split the data into traning and test data
  cv::Mat trainData, trainLabels, testData, testLabels;
  machine::split( data, labels, trainData, trainLabels, testData, testLabels, 0.7);
  std::cout<< "Traning samples: " << trainLabels.rows << std::endl;
  std::cout<< "Test samples: " << testData.rows << std::endl;
  std::cout<< "---" << std::endl;

  // Create the classifiers
  std::map<std::string, machine::MachinePtr> classifiers;
  classifiers.insert( std::pair<std::string, machine::MachinePtr>( "svm", machine::create("svm")) );
  classifiers.insert( std::pair<std::string, machine::MachinePtr>( "mlp", machine::create("mlp")) );
  classifiers.insert( std::pair<std::string, machine::MachinePtr>( "knn", machine::create("knn")) );
  classifiers.insert( std::pair<std::string, machine::MachinePtr>( "bayes", machine::create("bayes")) );

  // Prepare stats
  std::map<std::string, Stats> stats;
  stats.insert( std::pair<std::string, machine::Stats>( "svm", machine::Stats()) );
  stats.insert( std::pair<std::string, machine::Stats>( "mlp", machine::Stats()) );
  stats.insert( std::pair<std::string, machine::Stats>( "knn", machine::Stats()) );
  stats.insert( std::pair<std::string, machine::Stats>( "bayes", machine::Stats()) );
  
  // Run everything
  for ( uint x = 0; x < ite; x++) { 

    // Print intermediate stats
    if( (x+1) % 10 == 0 ) {
      std::cout << "____ Stats at iteration: " << (x+1) << " _____ " << std::endl;
    }
    
    // Train and test the classifers
    for( auto &classifier : classifiers) {
      stats[classifier.first].addMatrix(x);
      classifier.second->train( trainData, trainLabels);
      for( uint i = 0; i < testData.rows; i++) {
	cv::Mat sample = testData.row(i);
	float pred = classifier.second->predict(sample);
	if ( (int)pred == (int)testLabels.at<float>(i, 0) ) {
	  if( (int)pred > 0.5 ) {
	    stats[classifier.first].addTP(x, 1);
	  }
	  else {
	    stats[classifier.first].addTN(x, 1);
	  }
	}
	else {
	  if( (int)pred > 0.5 ) {
	    stats[classifier.first].addFP(x, 1);
	  }
	  else {
	    stats[classifier.first].addFN(x, 1);
	  }
	}
      }
      if( (x+1) % 10 == 0 ) {
	std::cout << "Accuracy (" << classifier.first <<"): " << stats[classifier.first].getAccuracy(x) << std::endl;
      }
    }

    // Reset the model each classifer
    for( auto &classifier : classifiers) {
      classifier.second = machine::create(classifier.first);
    }
    
    // Randomly make new split of training/test data...
    machine::split( data, labels, trainData, trainLabels, testData, testLabels, 0.7);
  }

  // Print the stats
  std::cout << std::endl << "___ Stats ____" << std::endl;
  for( auto &stat : stats) {
    std::cout << "Accuracy (" << stat.first <<"): " << stat.second.avgAccuracy() << " +/- " << stat.second.stdAccuracy() << std::endl;
    std::cout << "Balanced Accuracy (" << stat.first <<"): " << stat.second.avgBalancedAccuracy() << std::endl;
    std::cout << "F1 (" << stat.first <<"): " << stat.second.avgF1() << std::endl;
    /*
    std::cout << "TP: " << stats[stat.first]._stats[0]._TP << std::endl;
    std::cout << "FP: " << stats[stat.first]._stats[0]._FP << std::endl;
    std::cout << "TN: " << stats[stat.first]._stats[0]._TN << std::endl;
    std::cout << "FN: " << stats[stat.first]._stats[0]._FN << std::endl;
    */
    std::cout << "---------" << std::endl;
  }
}
