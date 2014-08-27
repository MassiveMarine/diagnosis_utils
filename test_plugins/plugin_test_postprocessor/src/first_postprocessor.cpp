#include <plugin_test_postprocessor/first_postprocessor.h>

namespace plugin_test_postprocessor
{
FirstPostprocessor::FirstPostprocessor()
{
}

void FirstPostprocessor::initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name)
{

}

bool FirstPostprocessor::process(const ros::Time& time, const ros::Duration& period)
{
//  ROS_INFO("postprocess()");
  return false;
}
}

PLUGINLIB_EXPORT_CLASS(plugin_test_postprocessor::FirstPostprocessor, tug_plugin_manager::RegularPlugin)
