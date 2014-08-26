#include <tug_robot_control/robot_hardware.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <controller_manager/controller_manager.h>
#include <tug_plugin_manager/plugin_manager.h>
#include <tug_plugin_manager/plugin_base.h>
#include <tug_controlled_device_driver/tug_controlled_device_driver_base.h>
#include <tug_preprocessor/tug_preprocessor_base.h>
#include <tug_postprocessor/tug_postprocessor_base.h>

#include <vector>

#define  LOOP_RATE 1.0

using tug_plugin_manager::PluginManager;
using tug_plugin_manager::PluginSpec;

using tug_controlled_device_driver::ControlledDeviceDriver;
typedef boost::shared_ptr<ControlledDeviceDriver> controlled_device_driver_ptr;
#define controlled_device_driver_ptr_cast(p) boost::dynamic_pointer_cast<ControlledDeviceDriver>(p)

using tug_preprocessor::Preprocessor;
typedef boost::shared_ptr<Preprocessor> preprocessor_ptr;
#define preprocessor_ptr_cast(p) boost::dynamic_pointer_cast<Preprocessor>(p)

using tug_postprocessor::Postprocessor;
typedef boost::shared_ptr<Postprocessor> postprocessor_ptr;
#define postprocessor_ptr_cast(p) boost::dynamic_pointer_cast<Postprocessor>(p)

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tug_robot_control_node");
  ros::NodeHandle nh;

  ROS_ERROR("Starting main");
  try
  {
    tug_robot_control::RobotHardware robot_hardware;
    controller_manager::ControllerManager cm(&robot_hardware);

    PluginManager* pm_device_driver = new PluginManager("tug_plugin_manager");
    PluginManager* pm_controlled_device_driver = new PluginManager("tug_plugin_manager");
    PluginManager* pm_preprocessor = new PluginManager("tug_plugin_manager");
    PluginManager* pm_postprocessor = new PluginManager("tug_plugin_manager");

    pm_device_driver->loadPlugin("1st device driver", "plugin_test_device_driver::FirstDeviceDriver");
    pm_controlled_device_driver->loadPlugin("1st controlled device driver",
        "plugin_test_controlled_device_driver::FirstControlledDeviceDriver");
    pm_controlled_device_driver->loadPlugin("2nd controlled device driver",
        "plugin_test_controlled_device_driver::FirstControlledDeviceDriver");
    pm_preprocessor->loadPlugin("First preprocessor", "plugin_test_preprocessor::FirstPreprocessor");
    pm_postprocessor->loadPlugin("First postprocessor", "plugin_test_postprocessor::FirstPostprocessor");

    ros::Rate loop_rate(LOOP_RATE);
    ros::Time last_time = ros::Time::now();

    ROS_ERROR("Starting main-loop");
    while (ros::ok())
    {
      loop_rate.sleep();

      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - last_time;
      last_time = current_time;

      std::vector<PluginSpec> controlled_device_driver_list = pm_controlled_device_driver->getPluginList();
      // read all controlled device driver
      for (unsigned int i = 0; i < controlled_device_driver_list.size(); i++)
      {
        controlled_device_driver_ptr driver = controlled_device_driver_ptr_cast(controlled_device_driver_list.at(i).instance);
        if (driver)
          driver->read(current_time, elapsed_time);
      }

      // do preprocessing work
      std::vector<PluginSpec> preprocessor_list = pm_preprocessor->getPluginList();
      for (unsigned int i = 0; i < preprocessor_list.size(); i++)
      {
        preprocessor_ptr processor = preprocessor_ptr_cast(preprocessor_list.at(i).instance);
        if (processor)
          processor->process(current_time, elapsed_time);
      }

      // make the update step
      cm.update(current_time, elapsed_time);

      // do postprocessing work
      std::vector<PluginSpec> postprocessor_list = pm_postprocessor->getPluginList();
      for (unsigned int i = 0; i < postprocessor_list.size(); i++)
      {
        postprocessor_ptr processor = postprocessor_ptr_cast(postprocessor_list.at(i).instance);
        if (processor)
          processor->process(current_time, elapsed_time);
      }

      // write all controlled device driver
      for (unsigned int i = 0; i < controlled_device_driver_list.size(); i++)
      {
        controlled_device_driver_ptr driver = controlled_device_driver_ptr_cast(controlled_device_driver_list.at(i).instance);
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
