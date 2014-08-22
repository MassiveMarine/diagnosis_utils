#include <plugin_test_preprocessor/first_preprocessor.h>

namespace plugin_test_preprocessor
{
FirstPreprocessor::FirstPreprocessor()
{
}
void FirstPreprocessor::initialize(std::string name)
{
  temp_data_ = name;
}
std::string FirstPreprocessor::getName()
{
  return temp_data_;
}
bool FirstPreprocessor::process(const ros::Time& time, const ros::Duration& period)
{
//  ROS_INFO("preprocess()");
  return false;
}
}

PLUGINLIB_EXPORT_CLASS(plugin_test_preprocessor::FirstPreprocessor, tug_plugin_manager::RegularPlugin)
