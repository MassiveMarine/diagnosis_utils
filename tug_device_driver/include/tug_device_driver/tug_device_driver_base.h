#ifndef TUG_DEVICE_DRIVER_BASE_H_
#define TUG_DEVICE_DRIVER_BASE_H_

#include <ros/ros.h>
#include <plugin_manager/plugin_base.h>

namespace plugin_base_device_driver
{
class DeviceDriver: public plugin_base::RegularPlugin
{
public:
  DeviceDriver(){};
};
}

#endif
