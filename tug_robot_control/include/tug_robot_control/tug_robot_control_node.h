#ifndef TUG_ROBOT_CONTROL_NODE_H
#define TUG_ROBOT_CONTROL_NODE_H

#include <tug_robot_control/robot_hardware.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <controller_manager/controller_manager.h>
#include <tug_plugin_manager/plugin_manager.h>
#include <tug_plugin_manager/plugin_base.h>

#include <tug_robot_control/robot_control_plugin_base.h>

#include <tug_robot_control/tug_controlled_device_driver_base.h>
#include <tug_robot_control/tug_preprocessor_base.h>
#include <tug_robot_control/tug_postprocessor_base.h>

#include <vector>

namespace tug_robot_control
{
using tug_plugin_manager::PluginManager;
using tug_plugin_manager::PluginSpec;

using tug_robot_control::PluginBase;
using tug_robot_control::ControlledDeviceDriver;
using tug_robot_control::Preprocessor;
using tug_robot_control::Postprocessor;

typedef boost::shared_ptr<PluginBase> PluginBasePtr;
typedef boost::shared_ptr<ControlledDeviceDriver> ControlledDeviceDriverPtr;
typedef boost::shared_ptr<Preprocessor> PreprocessorPtr;
typedef boost::shared_ptr<Postprocessor> PostprocessorPtr;

class RobotControl
{
public:
  RobotControl();
  virtual ~RobotControl();

  int run();

private:
  void loadAndInitPlugins();
  void runRobotControlLoop();

  void readControlledDevice(const ros::Time& time, const ros::Duration& period);
  void writeControlledDevice(const ros::Time& time, const ros::Duration& period);
  void readWriteControlledDevice(const ros::Time& time, const ros::Duration& period, bool write_mode = false);

  void doPreprocessing(const ros::Time& time, const ros::Duration& period);
  void doPostprocessing(const ros::Time& time, const ros::Duration& period);

  PluginBasePtr pluginBasePtrCast(tug_plugin_manager::RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<PluginBase>(ptr);
  }

  ControlledDeviceDriverPtr controlledDeviceDriverPtrCast(tug_plugin_manager::RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<ControlledDeviceDriver>(ptr);
  }

  PostprocessorPtr postprocessorPtrCast(tug_plugin_manager::RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<Postprocessor>(ptr);
  }

  PreprocessorPtr preprocessorPtrCast(tug_plugin_manager::RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<Preprocessor>(ptr);
  }

  ros::NodeHandle node_handle_;
  tug_robot_control::RobotHardware robot_hardware_;
  controller_manager::ControllerManager controller_manager_;

  PluginManager* pm_controlled_device_driver_;
  PluginManager* pm_device_driver_;
  PluginManager* pm_preprocessor_;
  PluginManager* pm_postprocessor_;

  boost::thread robot_control_loop_thread_;

};

}

#endif
