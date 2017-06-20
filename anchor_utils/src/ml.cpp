#include <numeric>

#include <anchor_utils/ml.hpp>
#include <anchor_utils/database.hpp>

// ---[ Namespace 
namespace ml {

  // ---[ Used namespaces ]---
  using namespace std;
  using namespace cv;
  using namespace mongo;


} // ...namespace ]---

// -------------------
// Namespace functions
// -------------------

// ---[ Common create function ]---
// ---------------------------------------
// This function will eventually by modified for extra param arguments
ml::MachinePtr ml::create(std::string type) {

}


// ---[ Read samples from the database ]---
void ml::read( std::string db_name, std::string collection, cv::Mat &data, cv::Mat &labels) {
  try { 

    // Read all ids
    std::vector<std::string> ids;
    mongo::Database::id_array( db_name, collection, ids);

    // Read each instance
    mongo::Database db( db_name, collection);
    for ( auto ite = ids.begin(); ite != ids.end(); ++ite) {

      // Read the samples
      std::vector<double> array;
      db.get<double>( *ite, "x", array);
      cv::Mat dataRow = cv::Mat( 1, array.size(), CV_32FC1);
      for( int i = 0; i < array.size(); i++) {
	dataRow.at<float>( 0, i) = (float)array[i];
      } 
      if( data.empty() ) { data = dataRow; }
      else { data.push_back(dataRow);      }

      // Read the label
      float y = (float)db.get<double>( *ite, "y");
      cv::Mat labelRow = cv::Mat( 1, 1, CV_32FC1);
      labelRow.at<float>(0,0) = y;
      if( labels.empty() ) { labels = labelRow; }
      else { labels.push_back(labelRow);        }      
    }
  }
  catch( const std::exception &e ) {
    std::cout << "[ml::read]" << e.what() << std::endl;
  }

}

// ---[Filter (remove) one attribute (column) of the samples ]---
void ml::filter(const cv::Mat &data, cv::Mat &filtered, int idx) {
  
  // Remove a attribute (column) given by indx
  for( uint i = 0; i < data.cols; i++) {
    if ( i == idx )
      continue;

    cv::Mat col = data.col(i);
    if ( filtered.empty() ) {
      filtered = col;
    }
    else {
      cv::hconcat( filtered, col, filtered);
    }
  }

}

// ---[ Create the training/testing split ]---
// ---------------------------------------------
// Default: 70-30[%] training-testing samples
// -----------------------------------------------
void ml::split( const cv::Mat &data, const cv::Mat &labels,
		cv::Mat &trainData, cv::Mat &trainLabels,
		cv::Mat &testData, cv::Mat &testLabels,
		float k) {

  // Create index vector
  std::vector<int> n(data.rows);
  std::iota( n.begin(), n.end(), 0);

  // Random generator
  cv::RNG rng( 0xFFFFFFFF );

  // Randomly split the total data collection
  while ( !n.empty() && (n.size() / (float)data.rows) > k ) {
    int i = rng.uniform(0, (int)n.size());
    testData.push_back(data.row(n[i]));
    testLabels.push_back(labels.row(n[i]));

    // Remove from index
    n.erase( n.begin() + i );
  } 

  // Assign the remaining samples...
  for ( auto ite = n.begin(); ite != n.end(); ++ite) {
    trainData.push_back(data.row(*ite));
    trainLabels.push_back(labels.row(*ite));    
  }
}

