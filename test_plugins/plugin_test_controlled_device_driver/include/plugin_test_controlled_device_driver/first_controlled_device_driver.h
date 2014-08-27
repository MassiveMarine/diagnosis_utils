#ifndef FIRST_CONTROLLED_DEVICE_DRIVER_H_
#define FIRST_CONTROLLED_DEVICE_DRIVER_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_robot_control/tug_controlled_device_driver_base.h>



namespace plugin_test_controlled_device_driver
{

class FirstControlledDeviceDriver: public tug_robot_control::ControlledDeviceDriver
{
public:
  FirstControlledDeviceDriver();

  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware, ros::NodeHandle & nh, std::string name);
  virtual void write(const ros::Time& time, const ros::Duration& period);
  virtual void read(const ros::Time& time, const ros::Duration& period);

private:

};
}

#endif
