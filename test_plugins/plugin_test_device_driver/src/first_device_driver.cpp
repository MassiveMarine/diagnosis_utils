#include <plugin_test_device_driver/first_device_driver.h>

namespace plugin_test_device_driver
{
FirstDeviceDriver::FirstDeviceDriver()
{
}
void FirstDeviceDriver::initialize(std::string name)
{
  temp_data_ = name;
}
std::string FirstDeviceDriver::getName()
{
  return temp_data_;
}
}

PLUGINLIB_EXPORT_CLASS(plugin_test_device_driver::FirstDeviceDriver, tug_plugin_manager::RegularPlugin)
