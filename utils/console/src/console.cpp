
#include <console/console.hpp>

// Console helper functions
namespace console {

  const double getDiffTimeSec(const ros::Time &t) {
    ros::Duration diff_time = ros::Time::now() - t;
    return diff_time.toSec();
  }
  
  const double getDiffTimeMsec(const ros::Time &t) {
    ros::Duration diff_time = ros::Time::now() - t;
    return (double)diff_time.toNSec() / 1000.0;
  }

  const double getDiffTimeNsec(const ros::Time &t) {
    ros::Duration diff_time = ros::Time::now() - t;
    return (double)diff_time.toNSec();
  }
}
