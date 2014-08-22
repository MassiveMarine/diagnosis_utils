#include <robot_plugins/first_robot.h>

namespace plugin_robot_ns
{
FirstRobotLoigge::FirstRobotLoigge()
{
}
void FirstRobotLoigge::initialize(std::string name)
{
  temp_data_ = name;
}
std::string FirstRobotLoigge::getName()
{
  return temp_data_;
}
}

PLUGINLIB_EXPORT_CLASS(plugin_robot_ns::FirstRobotLoigge, tug_plugin_manager::RegularPlugin)
