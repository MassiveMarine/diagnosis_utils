#ifndef TUG_DEVICE_DRIVER_BASE_H_
#define TUG_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>
#include <tug_plugin_manager/plugin_base.h>

namespace tug_device_driver
{
class DeviceDriver: public tug_plugin_manager::RegularPlugin
{
public:
  DeviceDriver(){};
};
}

#endif
