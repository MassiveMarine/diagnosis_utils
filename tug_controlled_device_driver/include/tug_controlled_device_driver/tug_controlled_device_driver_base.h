#ifndef TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_
#define TUG_CONTROLLED_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>
#include <plugin_manager/plugin_base.h>

namespace plugin_base_controlled_device_driver
{
class ControlledDeviceDriver: public plugin_base::RegularPlugin
{
public:
  ControlledDeviceDriver(){};
  virtual void write(const ros::Time& time, const ros::Duration& period) = 0;
  virtual void read(const ros::Time& time, const ros::Duration& period) = 0;
};
}

#endif
