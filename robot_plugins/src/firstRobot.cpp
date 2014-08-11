#include <pluginlib/class_list_macros.h>
#include <plugin_interface/plugin.h>
#include <plugin_interfaces/robot.h>


namespace robot_plugin_ns{

class FirstRobotLoigge : public plugin_interface::Plugin<plugin_interfaces::RobotInterface>
{
public:
  bool init(plugin_interfaces::RobotInterface* robot, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    ROS_ERROR("NEW Robot::init");

    return true;
  }

//  void update(const ros::Time& time, const ros::Duration& period)
//  {
//
//  }

//  void starting(const ros::Time& time) { ROS_ERROR("NEW Controller::starting"); }
//  void stopping(const ros::Time& time) { ROS_ERROR("NEW Controller::stopping"); }

private:
//  static const double gain_ = 1.25;
//  static const double setpoint_ = 3.00;
};
PLUGINLIB_EXPORT_CLASS(robot_plugin_ns::FirstRobotLoigge, plugin_interface::PluginBase)



}//namespace
