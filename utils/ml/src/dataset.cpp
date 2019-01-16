// ----------------
// Implementation of common namespace functions
// ----------------------------------------------------
#include <numeric>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ml/ml.hpp>
#include <database/database.hpp>


// ---[ Used namespaces ]---
using namespace std;
using namespace cv;


// ---[ Namespace.... 
namespace machine {

  // ---[ Read samples from the database ]---
  void read( string db_name, string collection, Mat &data, Mat &labels) {
    using namespace mongo;
    try { 

      // Read all ids
      std::vector<std::string> ids;
      Database::id_array( db_name, collection, ids);

      // Read each instance
      Database db( db_name, collection);
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
  void filter(const Mat &data, Mat &filtered, int idx) {
    Mat result;
  
    // Remove a attribute (column) given by indx
    for( uint i = 0; i < data.cols; i++) {
      if ( i == idx )
	continue;

      Mat col = data.col(i);
      if ( result.empty() ) {
	result = col;
      }
      else {
	cv::hconcat( result, col, result);
      }
    }
    filtered = result;
  }

  // ---[ Create the training/testing split ]---
  // ---------------------------------------------
  // Default: 70-30[%] training-testing samples
  // -----------------------------------------------
  void split( const cv::Mat &data, const cv::Mat &labels,
	      cv::Mat &trainData, cv::Mat &trainLabels,
	      cv::Mat &testData, cv::Mat &testLabels,
	      float k) {
    
    // Create index vector
    std::vector<int> n(data.rows);
    std::iota( n.begin(), n.end(), 0);

    // Random generator
    cv::RNG rng( time(NULL) );

    // Randomly split the total data collection
    if ( !testData.empty() ) {
      testData.resize(0);
      testLabels.resize(0);
    }
    while ( !n.empty() && (n.size() / (float)data.rows) > k ) {
      int i = rng.uniform(0, (int)n.size());
      testData.push_back(data.row(n[i]));
      testLabels.push_back(labels.row(n[i]));

      // Remove from index
      n.erase( n.begin() + i );
    } 

    // Assign the remaining samples...
    if ( !trainData.empty() ) {
      trainData.resize(0);
      trainLabels.resize(0);
    }
    for ( auto ite = n.begin(); ite != n.end(); ++ite) {
      trainData.push_back(data.row(*ite));
      trainLabels.push_back(labels.row(*ite));    
    }
  }

  // ---[ Evenly distribute the dataset ]---
  // ---------------------------------------------
  void distribute(const Mat &data, const Mat &labels,
			   Mat &distData, Mat &distLabels, int k) {
    //int cnt = cv::countNonZero(trainLabels);
    //std::cout << "Zeros: " <<  trainLabels.rows - cnt << std::endl;
    //std::cout << "Ones: " <<  cnt << std::endl;

    // Create index vector
    std::vector<int> n(data.rows);
    std::iota( n.begin(), n.end(), 0);

    // Clear existing data
    if ( !distData.empty() ) {
      distData.resize(0);
      distLabels.resize(0);
    }
  
    // Random generator
    cv::RNG rng( time(NULL) );

    // Randomly split the total data collection
    float dest = 0.0;
    while ( k > 0 ) {
      int i = rng.uniform(0, (int)n.size());
      if( (int)dest != (int)labels.at<float>(n[i], 0) )
	continue;

      // Add to dataset
      distData.push_back(data.row(n[i]));
      distLabels.push_back(labels.row(n[i]));
    
      // Remove from index
      n.erase( n.begin() + i );

      if( dest == 1.0 ) {
	dest = 0.0;
	k--;
      }
      else {
	dest += 1.0;
      }
    }
  }

  // ---[ Write the dataset to file (in JSON format) ]---
  // ---------------------------------------------
  void write2file( const Mat &data, const Mat &labels,
		   std::string f_name, std::string path) {

    ofstream of (path + "/" + f_name);
    if (of.is_open()) {
      of << "{\n";
      of << "  \"data\" : [\n";
      for( uint i = 0; i < data.rows; i++) {
	of << "    [";
	for( uint j = 0; j < data.cols; j++) {
	  of << setw(5) << data.at<float>(i,j);
	  if( j < data.cols - 1 ) { of << ", "; }
	}
	if( i < data.rows - 1 ) { of << "],\n"; }
	else                    { of << "]\n"; }
      }
      of << "  ],\n";
      of << "  \"targets\" : [\n";
      for( uint i = 0; i < labels.rows; i++) {
	of << "    " << setw(2) << labels.at<float>(0,i);
	if( i < labels.rows - 1 ) { of << ",\n"; }
      }
      of << "  ]\n}\n";
      // myfile << "This is a line.\n";
      //  myfile << "This is another line.\n";
      of.close();
    }
    else {
      cout << "[ml::write2file]" << " Unable to open file: " << path + "/" + f_name << endl;
    }
  }

} // ...namespace ]---
