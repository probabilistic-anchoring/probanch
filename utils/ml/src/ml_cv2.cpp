#include <numeric>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <ml/ml.hpp>
#include <database/database.hpp>


// ---[ Used namespaces ]---
using namespace std;
using namespace cv;

// ---[ Namespace 
namespace machine {

  // ---[ Main traning function ]---
  void ML::train( const Mat &data, const Mat &labels) {

    if ( this->_type == "svm" ) {
      CvSVM *svm_ptr = dynamic_cast<CvSVM*>(this->_model.get());
      svm_ptr->train( data, labels, Mat(), Mat(), this->_svm_p);
    }
    else if ( this->_type == "mlp" ) {
      CvANN_MLP *mlp_ptr = dynamic_cast<CvANN_MLP*>(this->_model.get());
      mlp_ptr->train( data, labels, Mat(), Mat(), this->_mlp_p);
    }
    else if ( this->_type == "knn" ) {
      CvKNearest *knn_ptr = dynamic_cast<CvKNearest*>(this->_model.get());
      knn_ptr->train( data, labels, Mat(), false, 3);
    }
    else if ( this->_type == "bayes" ) {
      CvNormalBayesClassifier *bayes_ptr = dynamic_cast<CvNormalBayesClassifier*>(this->_model.get());
      bayes_ptr->train( data, labels);
    }
    else if ( this->_type == "tree" ) {
      CvDTree *tree_ptr = dynamic_cast<CvDTree*>(this->_model.get());
      tree_ptr->train( data, CV_ROW_SAMPLE, labels, cv::Mat(), cv::Mat(), this->_var_type);
    }
    
  }

  // ---[ Main prediction function ]---
  float ML::predict(const Mat &sample) {
    float result = -1.0;
    
    if ( this->_type == "svm" ) {
      CvSVM *svm_ptr = dynamic_cast<CvSVM*>(this->_model.get());
      result = svm_ptr->predict(sample);
    }
    else if ( this->_type == "mlp" ) {
      CvANN_MLP *mlp_ptr = dynamic_cast<CvANN_MLP*>(this->_model.get());
      cv::Mat response(1, 1, CV_32FC1);
      mlp_ptr->predict(sample, response);
      result = response.at<float>(0, 0);
    }
    else if ( this->_type == "knn" ) {
      CvKNearest *knn_ptr = dynamic_cast<CvKNearest*>(this->_model.get());
      result = knn_ptr->find_nearest(sample, 3);
    }
    else if ( this->_type == "bayes" ) {
      CvNormalBayesClassifier *bayes_ptr = dynamic_cast<CvNormalBayesClassifier*>(this->_model.get());
      result = bayes_ptr->predict(sample);
    }
    else if ( this->_type == "tree" ) {
      CvDTree *tree_ptr = dynamic_cast<CvDTree*>(this->_model.get());
      CvDTreeNode* prediction = tree_ptr->predict(sample);
      result = prediction->value;
    }

    return result;
  }

  // ---[ Saves the model to database ]---
  void ML::save(const string db_name, const string collection) {
    using namespace mongo;
    try {

      // Write the model to string
      FileStorage fsWrite(".yml", FileStorage::WRITE + FileStorage::MEMORY);
      this->_model->write( *fsWrite, "");
      std::string buf = fsWrite.releaseAndGetString();

      // Create MongoDB document
      mongo::Database::Document doc;

      // Add the model (as string)
      doc.add<std::string>( "model", buf);
      doc.add<std::string>( "type", this->_type);

      // Commit all changes to database
      Database db( db_name, collection);
      db.insert(doc);
      
    }
    catch( const std::exception &e ) {
      std::cout << "[ml::save]" << e.what() << std::endl;
    }
  }

  // ---[ Saves the model to file ]---
  void ML::save(const string filename) {
    this->_model->save(filename.c_str());
  }
} // ...namespace ]---

// -------------------
// Namespace functions
// -------------------

// ---[ Create function ]---
// ---------------------------------------
machine::MachinePtr machine::create(const string type) {
  machine::MachinePtr ptr;

  shared_ptr<CvStatModel> model;
  if ( type == "svm" ) {

    // SVM parameters
    CvSVMParams params = CvSVMParams();
    params.svm_type = CvSVM::NU_SVC;
    params.nu = 0.1;
    params.kernel_type = CvSVM::RBF; //CvSVM::RBF, CvSVM::LINEAR ...
    //params.degree = 0; // for poly
    params.gamma = 20; // for poly/rbf/sigmoid
    //params.coef0 = 0; // for poly/sigmoid
    params.term_crit.type = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
    params.term_crit.max_iter = 10000;
    params.term_crit.epsilon = 1e-6;
    
    // Create the SVM classifier
    model = std::shared_ptr<CvStatModel>(new CvSVM());
    ptr = machine::MachinePtr(new ML( type, model, params));
  }
  else if( type == "mlp" ) {

    // MLP parameters
    CvANN_MLP_TrainParams params;
    CvTermCriteria criteria;
    criteria.max_iter = 100;
    criteria.epsilon = 0.00001f;
    criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
    params.train_method = CvANN_MLP_TrainParams::BACKPROP;
    params.bp_dw_scale = 0.01f;
    params.bp_moment_scale = 0.01f;
    params.term_crit = criteria;

    // Create the Network 
    cv::Mat layers = cv::Mat(4, 1, CV_32SC1);
    layers.row(0) = cv::Scalar(5);
    layers.row(1) = cv::Scalar(10); // 10
    layers.row(2) = cv::Scalar(15); // 15
    layers.row(3) = cv::Scalar(1);
    model = std::shared_ptr<CvStatModel>(new CvANN_MLP(layers));
    ptr = machine::MachinePtr(new ML( type, model, params));    
  }
  else if( type == "knn" ) {

    // Create the kNN classifier
    model = std::shared_ptr<CvStatModel>(new CvKNearest());
    ptr = machine::MachinePtr(new ML(type, model)); 
  }
  else if( type == "bayes" ) {

    // Create the Bayes classifier
    model = std::shared_ptr<CvStatModel>(new CvNormalBayesClassifier());
    ptr = machine::MachinePtr(new ML(type, model)); 
  }
  else if( type == "tree" ) {

    // Define attributes and output as numerical
    cv::Mat var_type(6, 1, CV_8U);
    var_type.at<unsigned int>(0,0) = CV_VAR_NUMERICAL;
    var_type.at<unsigned int>(0,1) = CV_VAR_NUMERICAL;
    var_type.at<unsigned int>(0,2) = CV_VAR_NUMERICAL;
    var_type.at<unsigned int>(0,3) = CV_VAR_NUMERICAL;
    var_type.at<unsigned int>(0,4) = CV_VAR_NUMERICAL;
    var_type.at<unsigned int>(0,5) = CV_VAR_NUMERICAL; // <-- output

    // Create the Decision tree classifier
    model = std::shared_ptr<CvStatModel>(new CvDTree());
    ptr = machine::MachinePtr(new ML(type, model, var_type)); 
  }
  else {
    std::cout << "[ml::create] Given type of machine learning algorithm is not supported." << std::endl;
  }

  return ptr;
}


// ---[ Load model from database ]---
machine::MachinePtr machine::load(const string db_name, const string collection, const string type) {

  using namespace mongo;
  machine::MachinePtr ptr;

  shared_ptr<CvStatModel> model;
  try { 
 
    // Load the model from the database
    Database db( db_name, collection);
    std::string id = db.get_id<std::string>( "type", type);    
    std::string buf = db.get<std::string>( id, "model");
    //std::cout << "Buff: " << buf << std::endl;
    
    // Read model from string
    FileStorage fsRead( buf, FileStorage::READ + FileStorage::MEMORY);
    if ( type == "svm" ) {
      model = std::shared_ptr<CvStatModel>(new CvSVM());
    }
    else if ( type == "mlp" ) {
      model = std::shared_ptr<CvStatModel>(new CvANN_MLP());
    }

    model->read( *fsRead, *(fsRead.getFirstTopLevelNode()) );
    ptr = machine::MachinePtr(new ML(type, model));

  }
  catch( const std::exception &e ) {
    std::cout << "[ml::load]" << e.what() << std::endl;
  }

  return ptr;
}

// ---[ Load model from file ]---
machine::MachinePtr machine::load(const string filename, const string type) {
  machine::MachinePtr ptr;

  if( type == "svm" ) {
    shared_ptr<CvStatModel> model = std::shared_ptr<CvStatModel>(new CvSVM());
    model->load(filename.c_str());
    ptr = machine::MachinePtr(new ML(type, model));  
  }
  else {
    std::cout << "[ml::load] Unable to load model of type: " << type << "." << std::endl;
  }
  
  return ptr;
}

