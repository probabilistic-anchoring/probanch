#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
//#include <pcl_ros/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <anchor_msgs/DisplayArray.h>
#include <anchor_msgs/LogicAnchorArray.h>
#include <geometry_msgs/Point.h>

//#include <tf2/convert.h>
//#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_eigen/tf2_eigen.h>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace cv;

class AnchorViewer {

  // ROS variables
  ros::NodeHandle _nh;
  ros::NodeHandle _priv_nh;
  image_transport::ImageTransport _it;
  ros::Subscriber _anchor_sub, _click_sub, _display_trigger_sub, _highlight_sub, _particle_sub;
  ros::Publisher _selected_pub;
  image_transport::Publisher _display_image_pub;

  // Display variables
  Mat _img, _icon;
  cv::Scalar _color;
  vector<anchor_msgs::Display> _anchors;
  int _counter;
  string _display_trigger, _highlight;
  bool _display_window;

  // Camera information
  image_geometry::PinholeCameraModel _cam_model;

  // Tranformation
  tf2::Stamped<tf2::Transform> _tf2;
  //tf2::Transform _tf;

  
  // --[ Private struct for storing a single particle ]--
  // -------------------------------------------------------
  struct Particle {
    double _x, _y, _z;
    string _color;

    Particle( double x, double y, double z, string color) :
      _x(x), _y(y), _z(z), _color(color) { }

    Particle(const Particle& p) :
      _x(p._x), _y(p._y), _z(p._z), _color(p._color) { }
    Particle& operator=(Particle p) {
      _x = p._x;
      _y = p._y;
      _z = p._z;
      _color = p._color;
      return *this;
    }

    Point2f getPixel( image_geometry::PinholeCameraModel &cam_model,
		      tf2::Transform &tf ) {
      tf2::Vector3 vec( _x, _y, _z);
      vec = tf.inverse() * vec;
      cv::Point3d pt_cv( vec.x(), vec.y(), vec.z());
      cv::Point2f p = cam_model.project3dToPixel(pt_cv);
      return p;
    };

    Scalar getColor() {
      Scalar c;
      if( _color == "white" ) c = Scalar( 242, 242, 242);       // white
      else if( _color == "gray" ) c = Scalar( 181, 181, 181);   // gray
      else if( _color == "black" ) c = Scalar( 27, 27, 27);     // black
      else if( _color == "magenta" ) c = Scalar( 252, 15, 252); // magenta
      else if( _color == "pink" ) c = Scalar( 253, 116, 252);   // pink
      else if( _color == "red" ) c = Scalar( 15, 15, 255);      // red
      else if( _color == "brown" ) c = Scalar( 42, 42, 165);    // brown
      else if( _color == "orange" ) c = Scalar( 15, 127, 255);  // orange
      else if( _color == "yellow" ) c = Scalar( 15, 255, 255);  // yellow
      else if( _color == "green" ) c = Scalar( 15, 255, 15);    // green
      else if( _color == "cyan" ) c = Scalar( 242, 242, 128);   // cyan
      else if( _color == "blue" ) c = Scalar( 255, 15, 15);     // blue
      else if( _color == "violet" ) c = Scalar( 255, 15, 143);  // violet
      else c = Scalar( 127, 15, 127);                           // purple
      return c;
    }
  };
  // --[ End of struct ]----

  // Private varaibles for storing particles
  vector<Particle> _particles;
  int _max_particles;
  cv::RNG _rng;

  
  // --[ Callback functions ] --
  
  // Callback function for handle web interface trigger
  void trigger_cb( const std_msgs::String::ConstPtr &msg) {
    if( msg->data == "anchoring" || msg->data == "logic" ) {
      this->_display_trigger = msg->data;
    }
    else {
      this->_display_trigger = "";
    }
  }

  // Callback function for reciving and displaying the image
  void display_cb(const anchor_msgs::DisplayArrayConstPtr& msg_ptr) {

    // Store the image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy( msg_ptr->image,
				    sensor_msgs::image_encodings::BGR8 );
      cv_ptr->image.copyTo(this->_img);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("[AnchorViewer::display_cb] Could not convert image.");
    }

    // Store the anchor information
    this->_anchors = msg_ptr->anchors;

    // Store the camera infromation
    this->_cam_model.fromCameraInfo(msg_ptr->info);

    // Stor the transformation
    tf2::fromMsg( msg_ptr->transform, this->_tf2);
    
    // Publish the resulting anchor image
    if( !this->_display_trigger.empty() ) {
      cv_ptr->image = this->anchor_img();
      cv_ptr->encoding = "bgr8";
      _display_image_pub.publish(cv_ptr->toImageMsg());
    }
    _counter++;
  }

  // Callback function for receiving particles from the logical reasoner
  void particles_cb(const anchor_msgs::LogicAnchorArrayPtr& msg_ptr) {
    
    this->_particles.clear();  // Clear all previosuly stored particles
    for( auto &anchor : msg_ptr->anchors ) {
      if( anchor.color.symbols.empty() )
        continue;
      auto p = anchor.particle_positions.begin();
      for( ; p != anchor.particle_positions.end(); ++p ) {
	this->_particles.push_back( Particle( p->data.pose.position.x, p->data.pose.position.y, p->data.pose.position.z, anchor.color.symbols.front()) );
      }
    }
    if( _max_particles >= 0 ) {
      while( this->_particles.size() > _max_particles ) {
	this->_particles.erase( this->_particles.begin() + this->_rng.uniform( 0, (int)this->_particles.size() ) );
      }
    }

    /*
    // Print log data
    for( auto it = _anchors.begin(); it != _anchors.end(); ++it) {
      ROS_INFO_STREAM("Anchor: " << it->x << " - position: (" << it->pos.pose.position.x << ", " << it->pos.pose.position.y << ", " << it->pos.pose.position.y << ")");
    }
    ROS_INFO_STREAM("-------------------------------");
    double x = 0.0, y = 0.0, z = 0.0, sz = (double)this->_particles.size();
    for( auto &p : this->_particles) {
      x += p._x;
      y += p._y;
      z += p._z;
    }
    ROS_INFO_STREAM("Mean position of particles: (" << x  / sz << ", " << y / sz << ", " << y / sz << ")");
    ROS_INFO_STREAM("-------------------------------");
    */
  }

  // Function for processing the scene of anchored objects
  cv::Mat anchor_img() {

    // Prepare result image(s)
    cv::Mat result_img, highlight_img;
    this->_img.copyTo(result_img);
    this->_img.copyTo(highlight_img);
    cv::cvtColor( result_img, result_img, CV_BGR2GRAY);
    cv::cvtColor( result_img, result_img, CV_GRAY2BGR);
    result_img.convertTo( result_img, -1, 1.0, 50);

    // Iterate anchors and draw result
    for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {

      if( ite->border.contour.empty() ) {
	continue;
      }

      // Get the contour
      std::vector<cv::Point> contour;
      for( uint i = 0; i < ite->border.contour.size(); i++) {
	cv::Point p( ite->border.contour[i].x, ite->border.contour[i].y );
	contour.push_back(p);
      }
      std::vector<std::vector<cv::Point> > contours;
      contours.push_back(contour);

      // Create an image mask
      cv::Mat mask( this->_img.size(), CV_8U, cv::Scalar(0) );
      cv::drawContours( mask, contours, -1, cv::Scalar(255), CV_FILLED);

      // Get the bounding box
      cv::Rect rect = cv::boundingRect(contour);
      rect = rect + cv::Size( 50, 50);  // ...add padding
      rect = rect - cv::Point( 25, 25);
      rect &= cv::Rect( cv::Point(0.0), this->_img.size()); // Saftey routine!

      // Draw the object
      this->_img.copyTo( result_img, mask);
      cv::drawContours( result_img, contours, -1, this->_color, 1);

      // Draw highlighted result
      if( ite->id == this->_highlight ) {
	cv::drawContours( highlight_img, contours, -1, cv::Scalar( 0, 255, 0), CV_FILLED);
	cv::drawContours( highlight_img, contours, -1, this->_color, 1);
      }

      // Print anchoring information
      if( this->_display_trigger == "anchoring" || this->_display_trigger == "both" ) {
	int x_top = rect.x + 5, y_top = rect.y + 5;
	int x_centre = x_top + this->_icon.cols / 2, y_centre = y_top + this->_icon.rows / 2;
	cv::line( result_img, cv::Point( x_centre, y_centre), cv::Point( x_centre + (rect.width * 0.8), y_centre), this->_color, 1);
	cv::circle( result_img, cv::Point( x_centre, y_centre), 10, cv::Scalar::all(255), -1);
	this->_icon.copyTo( result_img( cv::Rect( x_top, y_top, this->_icon.cols, this->_icon.rows) ) );
	cv::circle( result_img, cv::Point( x_centre, y_centre), 10, this->_color, 1);
	cv::putText( result_img, ite->x, cv::Point( rect.x + 22, rect.y + 12), cv::FONT_HERSHEY_DUPLEX, 0.4, this->_color, 1, 8);
      }

      // Draw particles
      if( this->_display_trigger == "logic" || this->_display_trigger == "both" ) {
	for( uint i = 0; i < this->_particles.size(); i++) {
	  cv::circle( result_img, this->_particles[i].getPixel(this->_cam_model, this->_tf2), 1, this->_particles[i].getColor(), -1);
	  cv::circle( highlight_img, this->_particles[i].getPixel(this->_cam_model, this->_tf2), 1, this->_particles[i].getColor(), -1);
	}

      }
    }
    cv::addWeighted( highlight_img, 0.2, result_img, 0.8, 0.0, result_img);
    /*
    cv::Rect roi( 280, 260, 360, 270);  // x, y, w, h
    cv::Mat crop = result_img(roi);     // Cropped images
    cv::rectangle( result_img, roi, color, 1);   
    */

    // Return the result
    return result_img;
  }

  // Callback function for reciving and displaying the image
  void highlight_cb(const std_msgs::StringConstPtr& msg) {
    this->_highlight = msg->data;
  }

  void help() {
    cout << "Usage:" << endl;
    cout << "----------------------" << endl;
    cout << "  r - Reset the target anchor." << endl;
    cout << "  h - Display this message." << endl;
    cout << "  q - Quit." << endl;
    cout << "----------------------" << endl;
  }

  // Dispaly (web interface) clikc callback function
  void web_interface_click_cb(const geometry_msgs::PointConstPtr& msg) {
    this->click_wrapper( cv::EVENT_LBUTTONDOWN, msg->x, msg->y, -1 );
  }
  
  // Mouse click callback function(s)
  static void mouse_click_cb(int event, int x, int y, int flags, void *obj) {
    AnchorViewer *self = static_cast<AnchorViewer*>(obj);
    if( event == cv::EVENT_LBUTTONDOWN ) {
      self->click_wrapper( event, x, y, flags);
    }
  }
  void click_wrapper(int event, int x, int y, int flags) {
    for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {

      // Get the contour
      std::vector<cv::Point> contour;
      for( uint i = 0; i < ite->border.contour.size(); i++) {
	cv::Point p( ite->border.contour[i].x, ite->border.contour[i].y );
	contour.push_back(p);
      }
      if( cv::pointPolygonTest( contour, cv::Point( x, y ), false) > 0 ) {
	this->_highlight = ite->id;
	ROS_WARN("Anchored selected at: x = %.2f, y = %.2f, z = %.2f", (float)ite->pos.pose.position.x, (float)ite->pos.pose.position.y, (float)ite->pos.pose.position.z);

	// Publish the id of the selected anchor
	std_msgs::String msg;
	msg.data = ite->id;
	_selected_pub.publish(msg);

	break;
      }
    }
  }

  // Generate a unique name
  std::string uname(const std::string &suffix) {

    // Get time now as a string
    boost::posix_time::ptime t = ros::Time::now().toBoost();
    std::string t_str = boost::posix_time::to_simple_string(t);

    // Use the time to create a uiquefile name
    std::replace( t_str.begin(), t_str.end(), ' ', '_');
    std::string name = t_str.substr( 0, t_str.find(".")) + "." + suffix;
    return name;
  }

public:
  AnchorViewer(ros::NodeHandle nh) : _nh(nh), _priv_nh("~"), _it(nh), _counter(0) { 

    // ROS subscriber/publisher
    _anchor_sub = _nh.subscribe("/display/anchors", 10, &AnchorViewer::display_cb, this);
    _particle_sub = nh.subscribe("/logic_anchors", 10, &AnchorViewer::particles_cb, this);
    _highlight_sub = _nh.subscribe("/display/selected", 10, &AnchorViewer::highlight_cb, this);
    _click_sub = _nh.subscribe("/display/click", 10, &AnchorViewer::web_interface_click_cb, this);
    _selected_pub = _nh.advertise<std_msgs::String>("/display/selected", 1);

    // Used for the web interface
    _display_trigger_sub = _nh.subscribe("/display/trigger", 1, &AnchorViewer::trigger_cb, this);
    _display_image_pub = _it.advertise("/display/image", 10);

    // Read ROS params
    if( !_priv_nh.getParam("max_particles", this->_max_particles) ) {
      this->_max_particles = -1;
    }
    if( !_priv_nh.getParam("display_mode", this->_display_trigger) ) {
      this->_display_trigger = "anchoring";
    }
    if( !_priv_nh.getParam("display_window", this->_display_window) ) {
      this->_display_window = true;
    }
    ROS_INFO("[AnchorViewer] Display mode: '%s'", this->_display_trigger.c_str());    
    ROS_INFO("[AnchorViewer] Max number of drawn particles: '%d'", this->_max_particles);    
    ROS_INFO_STREAM("[AnchorViewer] Display window: " << ( this->_display_window ? "Yes" : "No" ));
    
    // Set used color for drawing contoru and anchor information
    std::string color_name;
    if( !_priv_nh.getParam("draw_color", color_name) ) {
      color_name = "orange";
    }
    if( color_name == "white" )     this->_color = cv::Scalar::all(255);     // White
    else if( color_name == "red" )  this->_color = cv::Scalar( 0, 0, 233);   // Red
    else if( color_name == "gray" ) this->_color = cv::Scalar::all(64);      // Dark gray
    else                            this->_color = cv::Scalar( 32, 84, 233); // Orange (default)

    // Create the display window
    if( this->_display_window ) {
      const char *window = "Anchors with information...";
      cv::namedWindow( window, 1);
      cv::setMouseCallback( window, &AnchorViewer::mouse_click_cb, this);
    }

    // Read icon image
    string fname = ros::package::getPath("display") + "/images/symbol.png";
    this->_icon = cv::imread( fname, CV_LOAD_IMAGE_COLOR);
    cv::resize( this->_icon, this->_icon, cv::Size(), 0.06, 0.06, INTER_AREA);
      
    // Randomize
    this->_rng = cv::RNG( 0xFFFFFFFF );
  }
  ~AnchorViewer() {}

  void spin() {

    // Video recording varaibles
    bool recording = false;
    cv::VideoWriter video;
    string path = ros::package::getPath("display");

    // Main ROS loop
    ros::Rate rate(30);
    cv::Size size(0,0);
    while(ros::ok()) {

      // OpenCV window for display
      if( !this->_img.empty() && this->_display_window ) {
	cv::imshow( "Anchors with information...", this->anchor_img() );
	if( video.isOpened() ) {
	  video.write(this->anchor_img());
	}
	
	// Get the image size (once)
	if( size.width == 0 || size.height == 0 ) {
	  size = this->_img.size();
	}
      
	// Wait for a keystroke in the window
	char key = cv::waitKey(1);
	if( key == 27 || key == 'Q' || key == 'q' ) {
	  break;
	}
	else if( key == 'H' || key == 'h' ) {
	  this->help();
	}
	else if( key == 'R' || key == 'r' ) {
	  this->_highlight = "";
	}
	else if( key == 'X' || key == 'x' ) {
	  
	  // Save a screen-shot to file
	  cv::Mat cropped = this->anchor_img()( cv::Rect(280, 260, 360, 270) );
	  std::string name = this->uname("png");
	  cv::imwrite( path + "/images/" + name, cropped);
	  
	}
	else if( key == 'F' || key == 'f' ) {

	  // Log all anchors and their symbols to file.  
	  std::string name = this->uname("txt");
	  std::stringstream ss;
	  ss << "id:category:color:size" << endl;
	  std::ofstream oFile( path + "/images/" + name);
	  oFile  << ss.rdbuf();
	  for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {
	    ss.str("");
	    ss << ite->x << ":" << ite->category << ":" << ite->colors.front() << ":" << ite->size << endl;;
	    oFile  << ss.rdbuf();
	  }
	  oFile.close();
	}
	else if( key == 'S' || key == 's' ) {
	  recording = !recording;
	  if( recording ) {
	    
	    // Get a unique file name
	    std::string name = this->uname("avi"); 

	    // Start the recording
	    std::cout<< "[Start recording] File name: " << name << std::endl;
	    //video.open( path + name, CV_FOURCC('X','V','I','D'), 20, size, true);
	    video.open( path + "/videos/" + name, CV_FOURCC('M','J','P','G'), 30.0, size, true);
	  }
	  else {
	    std::cout<< "[Stop recording] Saved to path:" << path + "/videos/" << std::endl;
	    video.release();
	  }
	}
      }
      
      ros::spinOnce();
      rate.sleep();
    }

    // Closes all the windows
    cv::destroyAllWindows();
  }
};

// ------------------------------------------
// Main function
// ------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "anchor_viewer_node");
  ros::NodeHandle nh;
  AnchorViewer node(nh);
  node.spin();
  return 0;
}
