#ifndef TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_
#define TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>

#include <tug_robot_control/robot_hardware.h>
#include <tug_robot_control/robot_control_plugin_base.h>

namespace tug_robot_control
{
class ControlledDeviceDriver: public PluginBase
{
public:
  ControlledDeviceDriver(){};

  virtual void write(const ros::Time& time, const ros::Duration& period) = 0;
  virtual void read(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
