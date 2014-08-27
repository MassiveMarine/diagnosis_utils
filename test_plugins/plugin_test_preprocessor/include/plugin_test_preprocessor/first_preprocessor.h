#ifndef FIRST_PREPROCESSOR_H_
#define FIRST_PREPROCESSOR_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_robot_control/tug_preprocessor_base.h>

namespace plugin_test_preprocessor
{

class FirstPreprocessor: public tug_robot_control::Preprocessor
{
public:
  FirstPreprocessor();

//  virtual void initialize(std::string name);
//  virtual std::string getName();
  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware, ros::NodeHandle & nh, std::string name);

  virtual bool process(const ros::Time& time, const ros::Duration& period);

private:
//  std::string temp_data_;
};
}

#endif
