#include <tug_plugin_krikkit2/krikkit2.h>

namespace tug_plugin_krikkit2
{
  Krikkit2::Krikkit2()
  {
  }

  void Krikkit2::initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name)
  {
    ROS_INFO("Krikkit2::initialize");
  }

}

PLUGINLIB_EXPORT_CLASS(tug_plugin_krikkit2::Krikkit2, tug_plugin_manager::RegularPlugin)
