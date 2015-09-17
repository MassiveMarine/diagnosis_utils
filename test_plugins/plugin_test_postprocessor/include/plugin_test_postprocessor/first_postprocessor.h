#ifndef FIRST_POSTPROCESSOR_H_
#define FIRST_POSTPROCESSOR_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_robot_control/tug_postprocessor_base.h>

namespace plugin_test_postprocessor
{

class FirstPostprocessor: public tug_robot_control::Postprocessor
{
public:
  FirstPostprocessor();

  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name);

  virtual bool process(const ros::Time& time, const ros::Duration& period);

private:

};
}

#endif
