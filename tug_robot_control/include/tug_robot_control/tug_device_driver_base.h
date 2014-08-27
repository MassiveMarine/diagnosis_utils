#ifndef TUG_DEVICE_DRIVER_BASE_H_
#define TUG_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>
//#include <tug_plugin_manager/plugin_base.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
//class DeviceDriver: public tug_plugin_manager::RegularPlugin
class DeviceDriver: public PluginBase
{
public:
  DeviceDriver(){};
//  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware) = 0;
};
}

#endif
