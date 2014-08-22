//#include <test_robot/MyRobot.hpp>
#include <ros/console.h>
#include "ros/ros.h"
#include <controller_manager/controller_manager.h>
#include <tug_plugin_manager/plugin_manager.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_controlled_device_driver/tug_controlled_device_driver_base.h>
#include <tug_preprocessor/tug_preprocessor_base.h>
#include <tug_postprocessor/tug_postprocessor_base.h>

#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tug_robot_control_node");
  ros::NodeHandle nh;
  ROS_WARN("MAIN::starting");

  tug_plugin_manager::PluginManager* pm_controlled_device_driver = new tug_plugin_manager::PluginManager(
      "tug_plugin_manager");
  tug_plugin_manager::PluginManager* pm_device_driver = new tug_plugin_manager::PluginManager("tug_plugin_manager");
  tug_plugin_manager::PluginManager* pm_preprocessor = new tug_plugin_manager::PluginManager("tug_plugin_manager");
  tug_plugin_manager::PluginManager* pm_postprocessor = new tug_plugin_manager::PluginManager("tug_plugin_manager");

  pm_device_driver->loadPlugin("1st device driver", "plugin_test_device_driver::FirstDeviceDriver");
  pm_controlled_device_driver->loadPlugin("1st controlled device driver", "plugin_test_controlled_device_driver::FirstControlledDeviceDriver");
  pm_controlled_device_driver->loadPlugin("2nd controlled device driver", "plugin_test_controlled_device_driver::FirstControlledDeviceDriver");
  pm_preprocessor->loadPlugin("First preprocessor", "plugin_test_preprocessor::FirstPreprocessor");
  pm_postprocessor->loadPlugin("First postprocessor", "plugin_test_postprocessor::FirstPostprocessor");

  try
  {

//    MyRobot robot;
//    controller_manager::ControllerManager cm(&robot);

    ROS_WARN("MAIN::continue");

    double lr = 1.0;
    ros::Rate loop_rate(lr);
    ros::Time last_time = ros::Time::now();

    while (ros::ok())
    {
      loop_rate.sleep();

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - last_time;
      last_time = current_time;


      std::vector<tug_plugin_manager::PluginSpec> controlled_device_driver_list =
          pm_controlled_device_driver->getPluginList();
      // read all controlled device driver
      for (unsigned int i = 0; i < controlled_device_driver_list.size(); i++)
      {
        boost::shared_ptr<tug_controlled_device_driver::ControlledDeviceDriver> driver = boost::dynamic_pointer_cast<tug_controlled_device_driver::ControlledDeviceDriver>(controlled_device_driver_list.at(i).instance);
        if (driver)
          driver->read(current_time, elapsed_time);
      }

      // do preprocessing work
      std::vector<tug_plugin_manager::PluginSpec> preprocessor_list = pm_preprocessor->getPluginList();
      for (unsigned int i = 0; i < preprocessor_list.size(); i++)
      {
        boost::shared_ptr<tug_preprocessor::Preprocessor> processor = boost::dynamic_pointer_cast<
            tug_preprocessor::Preprocessor>(preprocessor_list.at(i).instance);
        if (processor)
          processor->process(current_time, elapsed_time);
      }

      // make the update step
//      cm.update(current_time, elapsed_time);

      // do postprocessing work
      std::vector<tug_plugin_manager::PluginSpec> postprocessor_list = pm_postprocessor->getPluginList();
      for (unsigned int i = 0; i < postprocessor_list.size(); i++)
      {
        boost::shared_ptr<tug_postprocessor::Postprocessor> processor = boost::dynamic_pointer_cast<
            tug_postprocessor::Postprocessor>(postprocessor_list.at(i).instance);
        if (processor)
          processor->process(current_time, elapsed_time);
      }

      // write all controlled device driver
      for (unsigned int i = 0; i < controlled_device_driver_list.size(); i++)
      {
        boost::shared_ptr<tug_controlled_device_driver::ControlledDeviceDriver> driver = boost::dynamic_pointer_cast<tug_controlled_device_driver::ControlledDeviceDriver>(controlled_device_driver_list.at(i).instance);
        if (driver)
          driver->write(current_time, elapsed_time);
      }
    }
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception!");
    return -1;
  }

  return 0;
}
