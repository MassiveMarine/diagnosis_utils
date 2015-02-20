#ifndef TUG_PLUGIN_KRIKKIT2_H_
#define TUG_PLUGIN_KRIKKIT2_H_

#include <pluginlib/class_list_macros.h>
#include <tug_plugin_manager/plugin_base.h>

// Select necessary plugin base
#include <tug_robot_control/tug_controlled_device_driver_base.h>
//#include <tug_robot_control/tug_device_driver_base.h>
//#include <tug_robot_control/tug_preprocessor_base.h>
//#include <tug_robot_control/tug_postprocessor_base.h>

namespace tug_plugin_krikkit2
{

class Krikkit2: public tug_robot_control::ControlledDeviceDriver
{
public:
  Krikkit2();

  virtual void initialize(tug_robot_control::RobotHardware* robot_hardware, const ros::NodeHandle & nh, std::string name);
  virtual void write(const ros::Time& time, const ros::Duration& period);
  virtual void read(const ros::Time& time, const ros::Duration& period);

private:

};
}

#endif /* TUG_PLUGIN_KRIKKIT2_H_ */
