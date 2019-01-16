#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <database/database.hpp>

using namespace mongo;

Database::Document compress(const cv::Mat &img) {

  // Compress the image  
  vector<uchar> buff;
  vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(90);
  cv::imencode( ".jpg", img, buff, compression_params);
  int length = (int)buff.size();

  // Return a sub document
  Database::Document doc;
  doc.add<int>( "rows", img.rows);
  doc.add<int>( "cols", img.cols);
  doc.add<unsigned char*>( "data", buff.data(), length);
  return doc;
}

cv::Mat decompress(const Database::Document &doc) {

  // Get image data
  int length = doc.get<std::size_t>("data");
  unsigned char* array = doc.get<unsigned char*>("data");

  // Decompress the image      
  vector<uchar> buff( array, array + length);
  cv::Mat img  = cv::imdecode( cv::Mat(buff), CV_LOAD_IMAGE_COLOR);

  return img;
}

void test_fn(const Database::Document &doc) {
  int length = doc.get<std::size_t>( "data");
  std::cout << "Array length: " << length << std::endl;
}


// --[ Main fn ]--- 
int main(int, char**) { 
  Database db("testdb", "testcollection");
  std::string id;
  
  // Query ids
  std::vector<std::string> ids;
  Database::id_array( "testdb", "testcollection", ids, "poses", true);
  for( auto id : ids ) {
    std::cout << "Id: " << id << ", will be removed." << std::endl;
    db.remove(id);
  }

  // Path to image on disk
  std::string path = ros::package::getPath("database");
  path = path + "/images/";
  cv::Mat img = cv::imread( path + "apple.jpg", CV_LOAD_IMAGE_COLOR);
  std::cout << "Image size: " << img.rows << " x " << img.cols << std::endl;

  // ----------------
  // Insert test
  // ----------------
  try {

    // Insert data 
    Database::Document doc;
    std::vector<double> poses = { 3.14, 2.42 };
    doc.add<double>( "poses", poses);
    doc.add<bool>( "trigger", false);

    Database::Document subdoc = compress(img);
    doc.add( "image", subdoc);

    id = db.insert(doc);
    std::cout << "Inserted: " << id << std::endl;
  }
  catch( const std::exception &e) {
    cout << e.what() << endl;
  }

  // ----------------
  // Update test
  // ----------------
  try {

    // Update simple types
    std::vector<double> poses = { 3.14, 2.42, 1.5 };
    db.update<double>( id, "poses", poses);
    db.update<bool>( id, "trigger", true);

    // Update a document
    Database::Document subdoc;
    subdoc.add<std::string>( "description", "An image of an apple.");
    subdoc.add<std::string>( "type", "RGB");
    db.update<Database::Document>( id, "info", subdoc);
  }
  catch( const std::exception &e) {
    cout << e.what() << endl;
  }

  
  // ----------------
  // Query test
  // ----------------
  try {

    Database::Document doc = db.get(id);

    // Get an array
    std::vector<double> poses;
    doc.get<double>( "poses", poses);
    std::cout << "Last position: " << poses.back() << std::endl;

    // Get image data
    std::string length = db.get<std::string>( id, "info.description");
    std::cout << "Image description: " << length << std::endl;

    // Get the image      
    Database::Document subdoc = doc.get("image");
    cv::Mat result = decompress(subdoc);
    std::cout << "Result size: " << result.rows << " x " << result.cols << std::endl;
    
    // Write to disk
    if( doc.get<bool>( "trigger" ) ) {
      cv::imwrite( path + "apple_from_db.jpg", result);
    }

  }
  catch( const std::exception &e) {
    cout << e.what() << endl;
  }
}
