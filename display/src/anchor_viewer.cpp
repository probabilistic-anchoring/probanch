#include <iostream>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <anchor_msgs/DisplayArray.h>
#include <anchor_msgs/LogicAnchorArray.h>
#include <geometry_msgs/Point.h>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace cv;

class AnchorViewer {

  /* --------------
     ROS variables
     -------------- */
  ros::NodeHandle _nh;
  ros::NodeHandle _priv_nh;
  image_transport::ImageTransport _it;
  ros::Subscriber _anchor_sub;
  ros::Subscriber _particle_sub;
  ros::Subscriber _highlight_sub;
  ros::Subscriber _click_sub;
  ros::Publisher _selected_pub;

  string _display_trigger;
  bool _display_window;
  ros::Subscriber _display_trigger_sub;
  image_transport::Publisher _display_image_pub;

  cv::Mat _img;
  vector<anchor_msgs::Display> _anchors;
  string _highlight;

  // Camera information
  image_geometry::PinholeCameraModel _cam_model;

  // Tranformation
  tf::Transform _tf;

  // Struct for storing a single particle
  struct Particle {
    double _x;
    double _y;
    double _z;
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
		      tf::Transform &tf ) {
      tf::Vector3 vec( _x, _y, _z);
      vec = tf * vec;
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
  vector<Particle> _particles;
  int _max_particles;
  cv::RNG _rng;

  // Callback fn for web interface trigger
  void trigger_cb( const std_msgs::String::ConstPtr &msg) {
    if( msg->data == "anchoring" || msg->data == "association" ) {
      this->_display_trigger = msg->data;
    }
    else {
      this->_display_trigger = "";
    }
    //this->_display_web_image = (msg->data == "anchoring") ? true : false;
  }

  // Callback function for reciving and displaying the image
  // TMP
  int _counter;
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
    tf::Quaternion tf_quat( msg_ptr->transform.rotation.x,
			    msg_ptr->transform.rotation.y,
			    msg_ptr->transform.rotation.z,
			    msg_ptr->transform.rotation.w );
    tf::Vector3 tf_vec( msg_ptr->transform.translation.x,
			msg_ptr->transform.translation.y,
			msg_ptr->transform.translation.z );
    this->_tf = tf::Transform( tf_quat, tf_vec);

    // Publish the resulting anchor image
    if( !this->_display_trigger.empty() ) {
      cv_ptr->image = this->anchor_img();
      cv_ptr->encoding = "bgr8";
      _display_image_pub.publish(cv_ptr->toImageMsg());
    }
    _counter++;
  }

  // Callback function for receiving particles from the data association
  void particles_cb(const anchor_msgs::LogicAnchorArrayPtr& msg_ptr) {
    //ROS_WARN("Got particles!");    
    
    this->_particles.clear();
    for( uint i = 0; i < msg_ptr->anchors.size(); i++) {
      auto p = msg_ptr->anchors[i].particle_positions.begin();
      //ROS_WARN("Number of particles: %d", (int)msg_ptr->anchors[i].particle_positions.size());
      for( ; p != msg_ptr->anchors[i].particle_positions.end(); ++p ) {
	this->_particles.push_back( Particle( p->data.pose.position.x, p->data.pose.position.y, p->data.pose.position.z, msg_ptr->anchors[i].color.symbols.front()) );
      }
    }
    if( _max_particles >= 0 ) {
      while( this->_particles.size() > _max_particles ) {
	this->_particles.erase( this->_particles.begin() + this->_rng.uniform( 0, (int)this->_particles.size() ) );
      }
    }
  }

  // Function for processing the scene image
  cv::Mat anchor_img() {

    // Draw the result
    cv::Mat result_img, highlight_img;
    this->_img.copyTo(result_img);
    this->_img.copyTo(highlight_img);
    //cv::Mat highlight_img(this->_img);
    cv::cvtColor( result_img, result_img, CV_BGR2GRAY);
    cv::cvtColor( result_img, result_img, CV_GRAY2BGR);
    result_img.convertTo( result_img, -1, 1.0, 50);

    // Set contour color
    //cv::Scalar color = cv::Scalar::all(255); // White
    cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
    //cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
    //cv::Scalar color = cv::Scalar::all(64); // Dark gray
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
      cv::drawContours( result_img, contours, -1, color, 1);
      /*
      // Draw contour or sub image
      if( this->_display_trigger == "association" ) {
	this->_img.copyTo( result_img, mask);
	cv::drawContours( result_img, contours, -1, color, 1);
      }
      else {
	//cv::Mat sub_img = result_img(rect);
	//this->_img.copyTo( result_img, rect);
	//sub_img.copyTo(result_img);
	this->_img(rect).copyTo( result_img(rect) );
      }
      */

      // Draw highlighted result
      if( ite->id == this->_highlight ) {
	cv::drawContours( highlight_img, contours, -1, cv::Scalar( 0, 255, 0), CV_FILLED);
	cv::drawContours( highlight_img, contours, -1, color, 1);
      }

      // Print anchoring information
      if( this->_display_trigger == "anchoring" || this->_display_trigger == "both" ) {
	cv::rectangle( result_img, rect, color, 1, 8);
	cv::putText( result_img, ite->x, cv::Point( rect.x+10, rect.y+16), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	/*
	 std::stringstream ss;
	 ss << "Symbol: " << ite->x;
	 cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 58), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 58), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 ss.str("");
	 ss << "Category: " << ite->category << " (" << ite->prediction * 100.0 << "%)";
	 cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 42), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 42), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 ss.str("");
	 ss << "Color(s): [";
	 for( uint i = 0; i < ite->colors.size(); i++) {
	   ss << ite->colors[i];
	   if( i < ite->colors.size() - 1 )
	     ss <<",";
	 }
	 ss << "]";
	 cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 26), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 26), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 ss.str("");
	 ss << "Size: " << ite->size;
	 cv::putText( result_img, ss.str(), cv::Point( rect.x, rect.y - 10), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 cv::putText( highlight_img, ss.str(), cv::Point( rect.x, rect.y - 10), cv::FONT_HERSHEY_DUPLEX, 0.4, color, 1, 8);
	 ss.str("");
	*/
      }

      // Draw particles
      if( this->_display_trigger == "association" || this->_display_trigger == "both" ) {
	for( uint i = 0; i < this->_particles.size(); i++) {
	  cv::circle( result_img, this->_particles[i].getPixel(this->_cam_model, this->_tf), 2, this->_particles[i].getColor(), -1);
	  cv::circle( highlight_img, this->_particles[i].getPixel(this->_cam_model, this->_tf), 2, this->_particles[i].getColor(), -1);
	}

      }
    }
    cv::addWeighted( highlight_img, 0.2, result_img, 0.8, 0.0, result_img);

    /*
    // TMP
    cv::Rect roi;
    roi.x = 80;
    roi.y = 200;
    roi.width = 320;
    roi.height = 320;
    cv::Mat crop = result_img(roi);
    cv::rectangle( crop, cv::Rect( 0, 0, crop.cols, crop.rows), cv::Scalar( 32, 84, 233), 2);
    cv::imwrite( "/home/aspo/testbed/images/" + std::to_string(_counter) + ".jpg", crop);
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

public:
  AnchorViewer(ros::NodeHandle nh) : _nh(nh), _priv_nh("~"), _it(nh) { //,  _display_trigger("anchoring"), _max_particles(-1) {
    _counter = 0;

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
      this->_display_window = false;
    }
    ROS_WARN("[AnchorViewer] Display mode: '%s'", this->_display_trigger.c_str());    
    ROS_WARN("[AnchorViewer] Number of particles: '%d'", this->_max_particles);    

    // Create the display window
    const char *window = "Anchors with information...";
    cv::namedWindow( window, 1);
    cv::setMouseCallback( window, &AnchorViewer::mouse_click_cb, this);

    this->_rng = cv::RNG( 0xFFFFFFFF );
  }
  ~AnchorViewer() {}

  void spin() {

    // Video recording varaibles
    bool recording = false;
    cv::VideoWriter video;
    string path = ros::package::getPath("display") + "/videos/";

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
	else if( key == 'S' || key == 's' ) {
	  recording = !recording;
	  if( recording ) {

	    // Get time now as a string
	    boost::posix_time::ptime t = ros::Time::now().toBoost();
	    std::string t_str = boost::posix_time::to_simple_string(t);

	    // Use the tim to create a uiquefile name
	    std::replace( t_str.begin(), t_str.end(), ' ', '_');
	    std::string name = t_str.substr( 0, t_str.find(".")) + ".avi";

	    // Start therecording
	    std::cout<< "[Start recording] File name: " << name << std::endl;
	    //video.open( path + name, CV_FOURCC('X','V','I','D'), 20, size, true);
	    video.open( path + name, CV_FOURCC('M','J','P','G'), 20, size, true);
	  }
	  else {
	    std::cout<< "[Stop recording] Saved to path:" << path << std::endl;
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
