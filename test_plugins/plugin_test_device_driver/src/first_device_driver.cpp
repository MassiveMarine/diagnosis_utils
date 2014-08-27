#include <plugin_test_device_driver/first_device_driver.h>

namespace plugin_test_device_driver
{
FirstDeviceDriver::FirstDeviceDriver()
{
}
//void FirstDeviceDriver::initialize(std::string name)
//{
//  temp_data_ = name;
//}
//std::string FirstDeviceDriver::getName()
//{
//  return temp_data_;
//}
void FirstDeviceDriver::initialize(tug_robot_control::RobotHardware* robot_hardware, ros::NodeHandle & nh, std::string name)
{

}
}

PLUGINLIB_EXPORT_CLASS(plugin_test_device_driver::FirstDeviceDriver, tug_plugin_manager::RegularPlugin)
