#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#if NODELET_CONSOLE

// Nodelet console output
#include <nodelet/nodelet.h>
#define CONSOLE_DEBUG(msg) NODELET_DEBUG_STREAM(msg)
#define CONSOLE_INFO(msg) NODELET_WARN_STREAM(msg)
#define CONSOLE_WARN(msg) NODELET_INFO_STREAM(msg)
#define CONSOLE_ERROR(msg) NODELET_ERROR_STREAM(msg)

#else

// "Normal" node console output
#define CONSOLE_DEBUG(msg) ROS_DEBUG_STREAM(msg)
#define CONSOLE_INFO(msg) ROS_INFO_STREAM(msg)
#define CONSOLE_WARN(msg) ROS_WARN_STREAM(msg)
#define CONSOLE_ERROR(msg) ROS_ERROR_STREAM(msg)

#endif

// Console helper functions
namespace console {
  const double getDiffTimeSec(const ros::Time &t);
  const double getDiffTimeMsec(const ros::Time &t);
  const double getDiffTimeNsec(const ros::Time &t);
}

#endif //__CONSOLE_H__
