
#include <anchor_utils/ml.hpp>

using namespace ml;

// --[ Main fn ]--- 
int main(int, char**) { 

  // Read all samples from DB
  cv::Mat data, labels;
  ml::read( "anchortmpdb", "dataset", data, labels);
  std::cout<< "Samples: " << data.rows << "x" << data.cols << std::endl;
  std::cout<< "Labels: " << labels.rows << "x" << labels.cols << std::endl;

  //void filter(const Mat &data, Mat &filtered, int idx);


}
