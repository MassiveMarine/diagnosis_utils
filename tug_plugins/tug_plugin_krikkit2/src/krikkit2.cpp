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

  void Krikkit2::write(const ros::Time& time, const ros::Duration& period)
  {
    ROS_INFO("Krikkit2::write()");
  }

  void Krikkit2::read(const ros::Time& time, const ros::Duration& period)
  {
    ROS_INFO("Krikkit2::read()");
  }
}

PLUGINLIB_EXPORT_CLASS(tug_plugin_krikkit2::Krikkit2, tug_plugin_manager::RegularPlugin)
