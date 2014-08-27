#ifndef TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_
#define TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>
//#include <tug_plugin_manager/plugin_base.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
//class ControlledDeviceDriver: public tug_plugin_manager::RegularPlugin
class ControlledDeviceDriver: public PluginBase
{
public:
  ControlledDeviceDriver(){};
//  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware) = 0;
  virtual void write(const ros::Time& time, const ros::Duration& period) = 0;
  virtual void read(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
