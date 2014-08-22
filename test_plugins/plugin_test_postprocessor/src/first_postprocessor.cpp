#include <plugin_test_postprocessor/first_postprocessor.h>

namespace plugin_test_postprocessor
{
FirstPostprocessor::FirstPostprocessor()
{
}
void FirstPostprocessor::initialize(std::string name)
{
  temp_data_ = name;
}
std::string FirstPostprocessor::getName()
{
  return temp_data_;
}
bool FirstPostprocessor::process(const ros::Time& time, const ros::Duration& period)
{
//  ROS_INFO("postprocess()");
  return false;
}
}

PLUGINLIB_EXPORT_CLASS(plugin_test_postprocessor::FirstPostprocessor, tug_plugin_manager::RegularPlugin)
