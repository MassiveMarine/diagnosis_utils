//#include <test_robot/MyRobot.hpp>
#include <ros/console.h>
#include "ros/ros.h"
//#include <controller_manager/controller_manager.h>
#include <tug_plugin_manager/plugin_manager.h>
#include <tug_plugin_manager/plugin_base.h>

#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_robot");
  ROS_WARN("MAIN::starting");

  tug_plugin_manager::PluginManager* pm_controlled_device_driver = new tug_plugin_manager::PluginManager("tug_plugin_manager");
  tug_plugin_manager::PluginManager* pm_device_driver = new tug_plugin_manager::PluginManager("tug_plugin_manager");
  tug_plugin_manager::PluginManager* pm_preprocessor = new tug_plugin_manager::PluginManager("tug_plugin_manager");
  tug_plugin_manager::PluginManager* pm_postprocessor = new tug_plugin_manager::PluginManager("tug_plugin_manager");
//
  pm_controlled_device_driver->loadPlugin("plugin_robot_ns::FirstRobotLoigge", "test1");
//  pm_robots->loadPlugin("plugin_robot_ns::FirstRobotLoigge", "test1");

//  const std::vector<std::string> names = pm_robots->getPluginNames();
//  ROS_WARN_STREAM("list all loaded plugins");
//  for (unsigned int i = 0; i < names.size(); i++)
//    ROS_INFO_STREAM(names.at(i));

//  const std::vector<plugin_manager::PluginSpec> plugin_list = pm_robots->getPluginList();
//  ROS_WARN_STREAM("list all loaded plugins");
//  for (unsigned int i = 0; i < plugin_list.size(); i++)
//    ROS_INFO_STREAM(plugin_list.at(i).name);

//  boost::shared_ptr<plugin_base::RegularPlugin> instance = pm_robots->getPluginInstanceByName("test1");
//  if (instance)
//    ROS_INFO_STREAM("get plugin by name: '" << instance->getName());
//  else
//    ROS_INFO_STREAM("no plugin");
  try
  {
//    ros::init(argc, argv, "test_robot");

//    MyRobot robot;
//    controller_manager::ControllerManager cm(&robot);

    ROS_WARN("MAIN::continue");

//    double lr = 1;
//    ros::Rate loop_rate(lr);
//    ros::Time last_time = ros::Time::now();

//    while (ros::ok())
//    {
//      loop_rate.sleep();
//
//      ros::Time current_time = ros::Time::now();
//      ros::Duration elapsed_time = current_time - last_time;
//      last_time = current_time;
//
//      robot.read(current_time, elapsed_time);
//      cm.update(current_time, elapsed_time);
//      robot.write(current_time, elapsed_time);
//    }
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception!");
    return -1;
  }

  return 0;
}
