#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <anchor_utils/database.hpp>

using namespace mongo;

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
  std::string path = "/home/aspo/testbed/src/anchoring/anchor_utils/images/";
  cv::Mat img = cv::imread( path + "apple.jpg", CV_LOAD_IMAGE_COLOR);
  std::cout << "Image size: " << img.rows << " x " << img.cols << std::endl;

  // ----------------
  // Insert test
  // ----------------
  try {

    // Compress the image  
    vector<uchar> buff;
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);
    cv::imencode( ".jpg", img, buff, compression_params);
    int length = (int)buff.size();

    std::vector<double> poses = { 3.14, 2.42 };

    // Insert data 
    db.prepare( P_INSERT );
    db.add<double>( "poses", poses);
    db.prepare( P_SUB_DOC );
    db.add<int>( "rows", img.rows);
    db.add<int>( "cols", img.cols);
    db.add<unsigned char*>( "data", buff.data(), length);
    db.add<PrepareType>( "image", P_SUB_DOC);

    id = db.insert();
    std::cout << "Inserted: " << id << std::endl;
  }
  catch( const std::exception &e) {
    cout << e.what() << endl;
  }
  
  // ----------------
  // Query test
  // ----------------
  try {

    // Get image data
    int length = db.get<std::size_t>( id, "image.data");
    unsigned char* array = db.get<unsigned char*>( id, "image.data");

    // Decompress the image      
    vector<uchar> buff( array, array + length);
    cv::Mat result  = cv::imdecode( cv::Mat(buff), CV_LOAD_IMAGE_COLOR);
    std::cout << "Result size: " << result.rows << " x " << result.cols << std::endl;
    
    // Write to disk
    cv::imwrite( path + "apple_2.jpg", result);
  }
  catch( const std::exception &e) {
    cout << e.what() << endl;
  }
  
}
