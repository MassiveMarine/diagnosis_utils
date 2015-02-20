#ifndef TUG_DEVICE_DRIVER_BASE_H_
#define TUG_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
class DeviceDriver: public PluginBase
{
public:
  DeviceDriver()
  {
  }

};
}

#endif
