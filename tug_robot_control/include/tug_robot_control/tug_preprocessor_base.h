#ifndef TUG_PREPROCESSOR_BASE_H_
#define TUG_PREPROCESSOR_BASE_H_

#include <ros/ros.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
class Preprocessor: public PluginBase
{
public:
  Preprocessor()
  {
  }

  virtual bool process(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
