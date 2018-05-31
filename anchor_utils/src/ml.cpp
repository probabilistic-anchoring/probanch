#include <numeric>

#include <anchor_utils/ml.hpp>
#include <anchor_utils/database.hpp>

// ---[ Used namespaces ]---
using namespace std;
using namespace cv;

// ---[ Namespace 
namespace machine {

  // ---[ Main traning function ]---
  void ML::train( const Mat &data, const Mat &labels) {

    // opencv2
#if CV_MAJOR_VERSION == 2
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

    // opencv3
#elif CV_MAJOR_VERSION == 3
    if ( this->_type == "svm" ) {
      Mat s_labels;
      labels.convertTo( s_labels, CV_32S);
      Ptr<ml::SVM> m = this->_model.dynamicCast<ml::SVM>();
      //m->train( data, ml::ROW_SAMPLE, s_labels );

      Ptr<ml::TrainData> d = ml::TrainData::create( data, cv::ml::ROW_SAMPLE, s_labels);
      m->trainAuto( d, 10 );
    }
#endif
    
  }

  // ---[ Main prediction function ]---
  float ML::predict(const Mat &sample) {
    float result = -1.0;
    
    // opencv2
#if CV_MAJOR_VERSION == 2
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

    // opencv3
#elif CV_MAJOR_VERSION == 3
    if ( this->_type == "svm" ) {
      cv::Mat response;
      Ptr<ml::SVM> m = this->_model.dynamicCast<ml::SVM>(); 
      m->predict( sample, response);
      result = response.at<float>(0, 0);
    }
#endif

    return result;
  }

  // ---[ Saves the model to database ]---
  void ML::save(string db_name, string collection) {
    using namespace mongo;
    try {

      // openc2
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
  void ML::save(string filename) {
    this->_model->save(filename.c_str());
  }
} // ...namespace ]---

// -------------------
// Namespace functions
// -------------------

// ---[ Common create function ]---
// ---------------------------------------
// This function will eventually by modified for extra param arguments
machine::MachinePtr machine::create(string type) {
  machine::MachinePtr ptr;

  // openc2
#if CV_MAJOR_VERSION == 2  
  shared_ptr<CvStatModel> model;
  if ( type == "svm" ) {

    /*
    // SVM parameters
    CvSVMParams params = CvSVMParams();
    params.svm_type = CvSVM::NU_SVC;
    params.kernel_type = CvSVM::RBF; //CvSVM::RBF, CvSVM::LINEAR ...
    params.degree = 0; // for poly
    params.gamma = 20; // for poly/rbf/sigmoid
    params.coef0 = 0; // for poly/sigmoid

    params.C = 7; // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
    params.nu = 0.1; // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
    params.p = 0.0; // for CV_SVM_EPS_SVR

    params.class_weights = NULL; // for CV_SVM_C_SVC
    params.term_crit.type = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
    params.term_crit.max_iter = 1000;
    params.term_crit.epsilon = 1e-6;
    */

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
    std::cout << "Given type of machine learning algorithm is not supported." << std::endl;;
  }

  // opencv3
#elif CV_MAJOR_VERSION == 3
  if ( type == "svm" ) {
    Ptr<ml::SVM> model = ml::SVM::create();

    // SVM parameters
    model->setType(ml::SVM::NU_SVC); 
    model->setKernel(ml::SVM::RBF); //CvSVM::RBF, CvSVM::LINEAR ...
    model->setGamma(20.0); // for poly/rbf/sigmoid
    model->setC(7.0);  // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
    model->setNu(0.1);  // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
    model->setP(0.0);  // for CV_SVM_EPS_SVR

    // Termination criterias
    cv::TermCriteria crit;
    crit.type = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
    crit.maxCount = 1000;
    crit.epsilon = 1e-6;
    model->setTermCriteria(crit);
    
    ptr = machine::MachinePtr(new ML(type, model)); 
  }  
  else {
    std::cout << "Given type of machine learning algorithm is not supported." << std::endl;;
  }
#endif

  
  return ptr;
}


// ---[ Load model from database ]---
machine::MachinePtr machine::load(string db_name, string collection, string type) {
  using namespace mongo;

  machine::MachinePtr ptr;
  // openc2
#if CV_MAJOR_VERSION == 2  
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
#endif
  return ptr;
}

machine::MachinePtr machine::load(string filename) {
  machine::MachinePtr ptr;

#if CV_MAJOR_VERSION == 2   // openc2    
  shared_ptr<CvStatModel> model = std::shared_ptr<CvStatModel>(new CvSVM());
  model->load(filename.c_str());
#elif CV_MAJOR_VERSION == 3 // opencv3
  Ptr<ml::SVM> model = cv::Algorithm::load<ml::SVM>(filename);
#endif  
  /*
  if ( type == "svm" ) {
    model = std::shared_ptr<CvStatModel>(new CvSVM());
  }
  else if ( type == "mlp" ) {
    model = std::shared_ptr<CvStatModel>(new CvANN_MLP());
  }
  */
  ptr = machine::MachinePtr(new ML("svm", model));
  return ptr;
}


// ---[ Read samples from the database ]---
void machine::read( string db_name, string collection, Mat &data, Mat &labels) {
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
void machine::filter(const Mat &data, Mat &filtered, int idx) {
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
void machine::split( const cv::Mat &data, const cv::Mat &labels,
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
void machine::distribute(const Mat &data, const Mat &labels,
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
