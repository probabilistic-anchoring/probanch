#include <iostream>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <anchor_msgs/DisplayArray.h>
#include <dc_msgs/ParticlePlot.h>

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
  ros::Publisher _selected_pub;

  string _display_trigger;
  //bool _display_web_image;
  ros::Subscriber _display_trigger_sub;
  image_transport::Publisher _display_image_pub;

  cv::Mat _img;
  vector<anchor_msgs::Display> _anchors;
  string _highlight;

  // Camera information
  image_geometry::PinholeCameraModel _cam_model;

  // Struct for storing a single particle
  struct Particle {
    double _x;
    double _y;
    double _z;
    string _color;

    Particle( double x, double y, double z, string color) :
      _x(x), _y(y), _z(z), _color(color) {}
    
    Point2f getPixel(image_geometry::PinholeCameraModel &cam_model) {
      cv::Point3d pt_cv( _x, _y, _z);
      cv::Point2f p = cam_model.project3dToPixel(pt_cv);
      cout << "Point: " << p.x << " - " << p.y << endl;
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

    // Publish the resulting anchor image
    if( !this->_display_trigger.empty() ) {
      cv_ptr->image = this->anchor_img();
      cv_ptr->encoding = "bgr8";
      _display_image_pub.publish(cv_ptr->toImageMsg());
    }
  }

  // Callback function for receiving particles from the data association
  void particles_cb(const dc_msgs::ParticlePlotConstPtr& msg_ptr) {
    for( uint i = 0; i < msg_ptr->color.size(); i++) {
      this->_particles.push_back( Particle( msg_ptr->x[i], msg_ptr->y[i], msg_ptr->z[i], msg_ptr->color[i]) );
    }
    if( this->_particles.size() > 100 ) {
      this->_particles.erase( this->_particles.begin() );
    }
  }

  // Function for processing the scene image 
  cv::Mat anchor_img() {

    // Draw the result
    cv::Mat result_img(this->_img);
    cv::Mat highlight_img(this->_img);
    /*
    cv::Mat result_img;
    cv::cvtColor( ithis->_img, result_img, CV_BGR2GRAY); 
    cv::cvtColor( result_img, result_img, CV_GRAY2BGR);
    result_img.convertTo( result_img, -1, 1.0, 50); 
    cv::Mat highlight_img(result_img);
    */
    for( auto ite = _anchors.begin(); ite != _anchors.end(); ++ite) {

      if( ite->border.contour.empty() ) {
	continue;
      }

      // Draw the contour
      std::vector<cv::Point> contour;
      for( uint i = 0; i < ite->border.contour.size(); i++) {
	cv::Point p( ite->border.contour[i].x, ite->border.contour[i].y );
	contour.push_back(p);
      }	  
      std::vector<std::vector<cv::Point> > contours;
      contours.push_back(contour);

      // Get the bounding box
      cv::Rect rect = cv::boundingRect(contour);
      
      // Draw highlighted result
      if( ite->id == this->_highlight ) {
	cv::drawContours( highlight_img, contours, -1, cv::Scalar( 0, 255, 0), CV_FILLED);
      }

      // Draw contour
      cv::Scalar color = cv::Scalar( 32, 84, 233); // Orange
      //cv::Scalar color = cv::Scalar( 0, 0, 233); // Red
      //cv::Scalar color = cv::Scalar::all(64); // Dark gray
      cv::drawContours( result_img, contours, -1, color, 1);
      cv::drawContours( highlight_img, contours, -1, color, 1);

      // Print anchoring infromation 
      if( this->_display_trigger == "anchoring" ) {
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
      }
      // Draw particles
      else if( this->_display_trigger == "association" ) {
	cout << "Particles: " << this->_particles.size() << endl;
	/*
	for( const auto &p : this->_particles) {
	  cv::circle( result_img, p.getPixel(this->_cam_model), 2, p.getColor(), -1);
	  cv::circle( highlight_img, p.getPixel(this->_cam_model), 2, p.getColor(), -1);
	}
	*/
	for( uint i = 0; i < this->_particles.size(); i++) {
	  cv::circle( result_img, this->_particles[i].getPixel(this->_cam_model), 2, this->_particles[i].getColor(), -1);
	  cv::circle( highlight_img, this->_particles[i].getPixel(this->_cam_model), 2, this->_particles[i].getColor(), -1);
	}

      }
    } 
    cv::addWeighted( highlight_img, 0.2, result_img, 0.8, 0.0, result_img);

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
  
  // Mouse click callback function(s)
  static void click_cb(int event, int x, int y, int flags, void *obj) {
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
  AnchorViewer(ros::NodeHandle nh) : _nh(nh), _priv_nh("~"), _it(nh),  _display_trigger("anchoring") { //_display_web_image(false) {
    
    // ROS subscriber/publisher
    _anchor_sub = _nh.subscribe("/display/anchors", 10, &AnchorViewer::display_cb, this);
    _particle_sub = nh.subscribe("/particles", 10, &AnchorViewer::particles_cb, this);1
    _highlight_sub = _nh.subscribe("/display/highlight", 10, &AnchorViewer::highlight_cb, this);
    _selected_pub = _nh.advertise<std_msgs::String>("/display/anchors", 1);

    // Used for the web interface
    _display_trigger_sub = _nh.subscribe("/display/trigger", 1, &AnchorViewer::trigger_cb, this);
    _display_image_pub = _it.advertise("/display/image", 10);

    
    const char *window = "Anchors with information...";
    cv::namedWindow( window, 1);
    cv::setMouseCallback( window, &AnchorViewer::click_cb, this);
  }
  ~AnchorViewer() {}

  void spin() {
    ros::Rate rate(30);
    while(ros::ok()) {

      // OpenCV window for display
      if( !this->_img.empty() ) {
	cv::imshow( "Anchors with information...", this->anchor_img() );
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

      ros::spinOnce();
      rate.sleep();
    }
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
