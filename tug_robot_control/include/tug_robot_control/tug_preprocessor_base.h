#ifndef TUG_PREPROCESSOR_BASE_H_
#define TUG_PREPROCESSOR_BASE_H_

#include <ros/ros.h>
//#include <tug_plugin_manager/plugin_base.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
//class Preprocessor: public tug_plugin_manager::RegularPlugin
class Preprocessor: public PluginBase
{
public:
  Preprocessor(){};
//  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware) = 0;
  virtual bool process(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
