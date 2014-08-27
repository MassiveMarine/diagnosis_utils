#ifndef ROBOT_CONTROL_PLUGIN_BASE_H_
#define ROBOT_CONTROL_PLUGIN_BASE_H_

#include <ros/ros.h>
#include <tug_plugin_manager/plugin_base.h>

#include <tug_robot_control/robot_hardware.h>

namespace tug_robot_control
{
class PluginBase: public tug_plugin_manager::RegularPlugin
{
public:
  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware, ros::NodeHandle & nh, std::string name) = 0;
protected:
  PluginBase()
  {
  }
};
}

#endif
