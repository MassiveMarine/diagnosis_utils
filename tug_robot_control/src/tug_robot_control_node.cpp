#include <tug_robot_control/tug_robot_control_node.h>

namespace tug_robot_control
{
RobotControl::RobotControl() :
    node_handle_("~"), robot_hardware_(), controller_manager_(&robot_hardware_, ros::NodeHandle(node_handle_, "plugins"))
{
  pm_controlled_device_driver_ = new PluginManager();
  pm_device_driver_ = new PluginManager();
  pm_preprocessor_ = new PluginManager();
  pm_postprocessor_ = new PluginManager();
}

int RobotControl::run()
{
  ROS_ERROR("Starting main");

  loadAndInitPlugins();

  robot_control_loop_thread_ = boost::thread(&RobotControl::runRobotControlLoop, this);

  ros::spin();
  robot_control_loop_thread_.join();
  return 0;

}

void RobotControl::loadAndInitPlugins()
{
  tug_plugin_manager::RegularPluginPtr new_plugin;
  new_plugin = pm_device_driver_->loadPlugin("1st device driver", "plugin_test_device_driver::FirstDeviceDriver");
  pluginBasePtrCast(new_plugin)->initialize(&robot_hardware_, node_handle_, "");

  new_plugin = pm_controlled_device_driver_->loadPlugin("1st controlled device driver",
      "plugin_test_controlled_device_driver::FirstControlledDeviceDriver");
  pluginBasePtrCast(new_plugin)->initialize(&robot_hardware_, node_handle_, "");

  new_plugin = pm_controlled_device_driver_->loadPlugin("2nd controlled device driver",
      "plugin_test_controlled_device_driver::FirstControlledDeviceDriver");
  pluginBasePtrCast(new_plugin)->initialize(&robot_hardware_, node_handle_, "");

  new_plugin = pm_preprocessor_->loadPlugin("First preprocessor", "plugin_test_preprocessor::FirstPreprocessor");
  pluginBasePtrCast(new_plugin)->initialize(&robot_hardware_, node_handle_, "");

  new_plugin = pm_postprocessor_->loadPlugin("First postprocessor", "plugin_test_postprocessor::FirstPostprocessor");
  pluginBasePtrCast(new_plugin)->initialize(&robot_hardware_, node_handle_, "");
}

void RobotControl::runRobotControlLoop()
{
  ROS_ERROR("Entering robot control loop");
  ros::Rate loop_rate(1.0);
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    try
    {
      // read all controlled device driver
      readControlledDevice(current_time, elapsed_time);

      // do preprocessing work
      doPreprocessing(current_time, elapsed_time);

      // make the update step
      controller_manager_.update(current_time, elapsed_time);

      // do postprocessing work
      doPostprocessing(current_time, elapsed_time);

      // write all controlled device driver
      writeControlledDevice(current_time, elapsed_time);

    }
    catch (std::exception & ex)
    {
      std::cerr << "Uncaught exception in robot control loop: " << ex.what() << std::endl;
    }
    catch (...)
    {
      std::cerr << "Uncaught exception in robot control loop" << std::endl;
    }
    loop_rate.sleep();
  }
  std::cout << "Exiting control loop" << std::endl;
}

void RobotControl::readControlledDevice(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpec> list = pm_controlled_device_driver_->getPluginList();
  for (unsigned int i = 0; i < list.size(); i++)
  {
    ControlledDeviceDriverPtr driver = controlledDeviceDriverPtrCast(list.at(i).instance);
    if (driver)
      driver->read(time, period);
  }
}
void RobotControl::writeControlledDevice(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpec> list = pm_controlled_device_driver_->getPluginList();
  for (unsigned int i = 0; i < list.size(); i++)
  {
    ControlledDeviceDriverPtr driver = controlledDeviceDriverPtrCast(list.at(i).instance);
    if (driver)
      driver->write(time, period);

  }
}

void RobotControl::doPreprocessing(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpec> postprocessor_list = pm_postprocessor_->getPluginList();
  for (unsigned int i = 0; i < postprocessor_list.size(); i++)
  {
    PostprocessorPtr processor = postprocessorPtrCast(postprocessor_list.at(i).instance);
    if (processor)
      processor->process(time, period);
  }
}
void RobotControl::doPostprocessing(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpec> preprocessor_list = pm_preprocessor_->getPluginList();
  for (unsigned int i = 0; i < preprocessor_list.size(); i++)
  {
    PreprocessorPtr processor = preprocessorPtrCast(preprocessor_list.at(i).instance);
    if (processor)
      processor->process(time, period);
  }
}

RobotControl::~RobotControl()
{
}
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "tug_robot_control_node");
    tug_robot_control::RobotControl node;
    return node.run();
  }
  catch (std::exception & ex)
  {
    std::cerr << "Uncaught exception in main: " << ex.what() << std::endl;
    return -1;
  }
  catch (...)
  {
    std::cerr << "Uncaught exception in main" << std::endl;
    throw;
    return -1;

  }
  return -1;
}
