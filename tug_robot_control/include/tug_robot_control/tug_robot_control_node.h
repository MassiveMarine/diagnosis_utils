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
using tug_plugin_manager::RegularPlugin;

using tug_robot_control::PluginBase;
using tug_robot_control::ControlledDeviceDriver;
using tug_robot_control::Preprocessor;
using tug_robot_control::Postprocessor;

typedef boost::shared_ptr<PluginBase> PluginBasePtr;
typedef boost::shared_ptr<ControlledDeviceDriver> ControlledDeviceDriverPtr;
typedef boost::shared_ptr<Preprocessor> PreprocessorPtr;
typedef boost::shared_ptr<Postprocessor> PostprocessorPtr;

typedef boost::shared_ptr<PluginManager<RegularPlugin> > PluginManagerPtr;
typedef boost::shared_ptr<RegularPlugin>  RegularPluginPtr;
typedef PluginSpec<RegularPlugin> PluginSpecInst;

class RobotControl
{
public:
  RobotControl();
  virtual ~RobotControl();

  void init();
  int run();

private:
  void loadAndInitPlugins();
  void loadAndInitPlugin(PluginManagerPtr pm, std::string prefix);

  void runRobotControlLoop();

  void readControlledDevice(const ros::Time& time, const ros::Duration& period);
  void writeControlledDevice(const ros::Time& time, const ros::Duration& period);
  void readWriteControlledDevice(const ros::Time& time, const ros::Duration& period, bool write_mode = false);

  void doPostprocessing(const ros::Time& time, const ros::Duration& period);
  void doPreprocessing(const ros::Time& time, const ros::Duration& period);

  PluginBasePtr pluginBasePtrCast(RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<PluginBase>(ptr);
  }

  ControlledDeviceDriverPtr controlledDeviceDriverPtrCast(RegularPluginPtr ptr)
  {
    ROS_DEBUG("controlledDeviceDriverPtrCast called");
    ROS_DEBUG_STREAM("cast pointer " << ptr << " to controlled device driver");
    return boost::dynamic_pointer_cast<ControlledDeviceDriver>(ptr);
  }

  PostprocessorPtr postprocessorPtrCast(RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<Postprocessor>(ptr);
  }

  PreprocessorPtr preprocessorPtrCast(RegularPluginPtr ptr)
  {
    return boost::dynamic_pointer_cast<Preprocessor>(ptr);
  }

  ros::NodeHandle node_handle_;
  double robot_control_loop_rate_;
  tug_robot_control::RobotHardware robot_hardware_;
  controller_manager::ControllerManager controller_manager_;

  PluginManagerPtr pm_controlled_device_driver_;
  PluginManagerPtr pm_device_driver_;
  PluginManagerPtr pm_postprocessor_;
  PluginManagerPtr pm_preprocessor_;

  boost::thread robot_control_loop_thread_;

};

}

#endif
