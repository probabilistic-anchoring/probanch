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
        
    // Prepare the dataset
    Mat s_labels;
    if ( this->_type == "mlp" ) {
      s_labels = Mat::zeros( labels.rows, 2, CV_32F);
      for( uint i = 0; i < labels.rows; i++) {
	if( labels.at<float>(i, 0) > 0.0 )
	  s_labels.at<float>(i, 1) = 1.0;
	else 
	  s_labels.at<float>(i, 0) = 1.0;
      }
    }
    else {
      labels.convertTo( s_labels, CV_32S);  
    }
    Ptr<ml::TrainData> dataset = ml::TrainData::create( data, ml::SampleTypes::ROW_SAMPLE, s_labels);
    
    // Specify the layer configuration for ANN (MLP)
    if ( this->_type == "mlp" ) {
      const int hiddenLayers1 = 10;
      const int hiddenLayers2 = 15;
      const int hiddenLayers = 50;
      Ptr<ml::ANN_MLP> model = this->_model.dynamicCast<ml::ANN_MLP>();

      Mat layerSizes = Mat(4, 1, CV_16U);
      layerSizes.row(0) = Scalar(data.cols);
      layerSizes.row(1) = Scalar(hiddenLayers1);
      layerSizes.row(2) = Scalar(hiddenLayers2);
      //layerSizes.row(1) = Scalar(hiddenLayers);
      layerSizes.row(3) = Scalar(s_labels.cols);
      //layerSizes.row(2) = Scalar(s_labels.cols);
      model->setLayerSizes(layerSizes);
    }
    
    // Train the model
    if ( this->_type == "svm" ) {
      Ptr<ml::SVM> model = this->_model.dynamicCast<ml::SVM>();
      model->trainAuto( dataset, 16 );
    }
    else if ( this->_type == "mlp" ) {
      this->_model->train( dataset,
			   ml::ANN_MLP::TrainFlags::NO_INPUT_SCALE +
			   ml::ANN_MLP::TrainFlags::NO_OUTPUT_SCALE  );
    }
    else {
      this->_model->train( dataset );
    }
    
  }

  // ---[ Main prediction function ]---
  float ML::predict(const Mat &sample) {
    cv::Mat response;
    if( this->_type == "knn" ) {
      Ptr<ml::KNearest> model = this->_model.dynamicCast<ml::KNearest>();
      model->findNearest(sample, 3, response);
    }
    else if( this->_type == "bayes" ) {
      cv::Mat probabilities;
      Ptr<ml::NormalBayesClassifier> model = this->_model.dynamicCast<ml::NormalBayesClassifier>();
      model->predictProb( sample, response, probabilities);
      return (float)response.at<int>(0, 0);
    }
    else {
      this->_model->predict( sample, response);
    }
    
    if ( this->_type == "mlp" ) {
      return response.at<float>(0, 0) > response.at<float>(0, 1) ? 0.0 : 1.0; 
    }
    return response.at<float>(0, 0);
  }

  // ---[ Saves the model to database ]---
  void ML::save(const string db_name, const string collection) {
    using namespace mongo;
    try {

      /*** ONLY CV2 SUPPORT?? ****/
#if CV_MAJOR_VERSION == 2
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
#endif
      
    }
    catch( const std::exception &e ) {
      std::cout << "[ml::save]" << e.what() << std::endl;
    }
  }

  // ---[ Saves the model to file. ]---
  void ML::save(const string filename) {
    this->_model->save(filename.c_str());
  }
} // ...namespace ('ml') ]---

// -------------------
// Namespace functions
// -------------------

// ---[ Create function ]---
// ---------------------------------------
machine::MachinePtr machine::create(const string type) {
  
  // Check if the type is supported
  assert( type != "svm" or type != "mlp" or type != "knn" or type != "bayes" );

  // Create the model
  machine::MachinePtr ptr;
  if ( type == "svm" ) {
    Ptr<ml::SVM> model = ml::SVM::create();

    // SVM parameters
    model->setType(ml::SVM::NU_SVC); 
    model->setKernel(ml::SVM::INTER); // ml::SVM::RBF, ml:SVM::LINEAR ...
    model->setGamma(0.5); // for poly/rbf/sigmoid (or 10.0?)
    model->setC(7.0);  // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
    model->setNu(0.1);  // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
    //model->setP(0.1);  // for CV_SVM_EPS_SVR

    // Termination criterias
    TermCriteria crit;
    crit.type = TermCriteria::Type::COUNT + TermCriteria::Type::EPS;
    crit.maxCount = 1000;
    crit.epsilon = 1e-6;
    model->setTermCriteria(crit);

    ptr = machine::MachinePtr(new ML(type, model));
  }
  else if ( type == "mlp" ) {
    Ptr<ml::ANN_MLP> model = ml::ANN_MLP::create();

    //model->setActivationFunction(ml::ANN_MLP::ActivationFunctions::SIGMOID_SYM);
    model->setActivationFunction(ml::ANN_MLP::ActivationFunctions::SIGMOID_SYM, 1.0, 1.0);

    //model->setTrainMethod(ANN_MLP::TrainingMethods::BACKPROP, 0.05, 0.05);
    model->setTrainMethod(ml::ANN_MLP::BACKPROP);
    model->setBackpropWeightScale(0.01f);
    model->setBackpropMomentumScale(0.01f);

    // Termination criterias
    TermCriteria crit;
    crit.type = TermCriteria::Type::COUNT + TermCriteria::Type::EPS;
    crit.maxCount = 1000;
    crit.epsilon = 1e-6;
    model->setTermCriteria(crit);
    
    ptr = machine::MachinePtr(new ML(type, model));  
  }
  else if ( type == "knn" ) {
    Ptr<ml::KNearest> model = ml::KNearest::create();
    model->setDefaultK(3);
    ptr = machine::MachinePtr(new ML(type, model));
  }
  else if ( type == "bayes" ) {
    Ptr<ml::NormalBayesClassifier> model = ml::NormalBayesClassifier::create();
    ptr = machine::MachinePtr(new ML(type, model));
  }  

  // Return the "machine" pointer
  return ptr;
}


// ---[ Load model from database ]---
machine::MachinePtr machine::load(const string db_name, const string collection, const string type) {

  using namespace mongo;
  machine::MachinePtr ptr;
  
  try { 
 
    
      /*** ONLY CV2 SUPPORT?? ****/
#if CV_MAJOR_VERSION == 2      // Load the model from the database
    shared_ptr<CvStatModel> model;
    
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
#endif
    
  }
  catch( const std::exception &e ) {
    std::cout << "[ml::load]" << e.what() << std::endl;
  }

  return ptr;
}

machine::MachinePtr machine::load(const string filename, const string type) {

  // Check if the type is supported
  assert( type != "svm" or type != "mlp" );

  // Load the model
  Ptr<ml::StatModel> model;
  if( type == "svm" ) {
    model = cv::Algorithm::load<ml::SVM>(filename);
  }
  else if( type == "mlp" ) {
    model = cv::Algorithm::load<ml::ANN_MLP>(filename);
  }
  
  // Create and return the "machine" pointer
  machine::MachinePtr ptr = machine::MachinePtr(new ML(type, model));
  return ptr;
}

