#include <plugin_test_controlled_device_driver/first_controlled_device_driver.h>

namespace plugin_test_controlled_device_driver
{
FirstControlledDeviceDriver::FirstControlledDeviceDriver()
{
}

void FirstControlledDeviceDriver::initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name)
{
  int data1;
  if(!nh.getParam("data1",data1))
    throw std::runtime_error("no 'data1' parameter");

  ROS_INFO_STREAM("reading 'data1' of '"<< data1 << "'");

}

void FirstControlledDeviceDriver::write(const ros::Time& time, const ros::Duration& period)
{
//  ROS_INFO("write()");
}

void FirstControlledDeviceDriver::read(const ros::Time& time, const ros::Duration& period)
{
//  ROS_INFO("read()");
}
}

PLUGINLIB_EXPORT_CLASS(plugin_test_controlled_device_driver::FirstControlledDeviceDriver, tug_plugin_manager::RegularPlugin)
