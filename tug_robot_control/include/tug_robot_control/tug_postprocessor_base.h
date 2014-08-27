#ifndef TUG_POSTPORCESSOR_BASE_H_
#define TUG_POSTPROCESSOR_BASE_H_

#include <ros/ros.h>
//#include <tug_plugin_manager/plugin_base.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
//class Postprocessor: public tug_plugin_manager::RegularPlugin
class Postprocessor: public PluginBase
{
public:
  Postprocessor(){};
//  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware) = 0;
  virtual bool process(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
