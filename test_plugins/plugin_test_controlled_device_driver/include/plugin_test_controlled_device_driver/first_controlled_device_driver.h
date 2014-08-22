#ifndef FIRST_CONTROLLED_DEVICE_DRIVER_H_
#define FIRST_CONTROLLED_DEVICE_DRIVER_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>
//#include <tug_device_driver/tug_device_driver_base.h>
#include <tug_controlled_device_driver/tug_controlled_device_driver_base.h>
//#include <tug_preprocessor/tug_preprocessor_base.h>
//#include <tug_postprocessor/tug_postprocessor_base.h>

namespace plugin_test_controlled_device_driver
{

class FirstControlledDeviceDriver: public tug_controlled_device_driver::ControlledDeviceDriver
{
public:
  FirstControlledDeviceDriver();

  virtual void initialize(std::string name);
  virtual std::string getName();
  virtual void write(const ros::Time& time, const ros::Duration& period);
  virtual void read(const ros::Time& time, const ros::Duration& period);

private:
  std::string temp_data_;
};
}

#endif
