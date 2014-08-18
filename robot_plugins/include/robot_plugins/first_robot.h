#ifndef ROBOT_PLUGINS_FIRST_ROBOT_BASE_H_
#define ROBOT_PLUGINS_FIRST_ROBOT_BASE_H_

#include <pluginlib/class_list_macros.h>
#include <plugin_manager/plugin_base.h>
#include <tug_device_driver/tug_device_driver_base.h>
//#include <tug_controlled_device_driver/tug_controlled_device_driver_base.h>
//#include <tug_preprocessor/tug_preprocessor_base.h>
//#include <tug_postprocessor/tug_postprocessor_base.h>

namespace plugin_robot_ns
{

class FirstRobotLoigge: public plugin_base_device_driver::DeviceDriver
{
public:
  FirstRobotLoigge();

  virtual void initialize(std::string name);
  virtual std::string getName();

private:
  std::string temp_data_;
};
}

#endif
