#ifndef TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_
#define TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>
#include <tug_plugin_manager/plugin_base.h>

#include <hardware_interface/robot_hw.h>

namespace tug_controlled_device_driver
{
class ControlledDeviceDriver: public tug_plugin_manager::RegularPlugin, hardware_interface::RobotHW
{
public:
  ControlledDeviceDriver(){};
  virtual void write(const ros::Time& time, const ros::Duration& period) = 0;
  virtual void read(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
