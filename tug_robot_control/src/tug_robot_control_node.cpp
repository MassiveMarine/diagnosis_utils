#include <tug_robot_control/tug_robot_control_node.h>

namespace tug_robot_control
{
RobotControl::RobotControl() :
    node_handle_("~"),
    robot_hardware_(),
    controller_manager_(&robot_hardware_,ros::NodeHandle(node_handle_, "controllers")),
    pm_controlled_device_driver_(new PluginManager()),
    pm_device_driver_(new PluginManager()),
    pm_preprocessor_(new PluginManager()),
    pm_postprocessor_(new PluginManager())
{
}

void RobotControl::init()
{
  node_handle_.param<double>("robot_control_loop_rate", this->robot_control_loop_rate_,25.0);
}

int RobotControl::run()
{
  loadAndInitPlugins();

  robot_control_loop_thread_ = boost::thread(&RobotControl::runRobotControlLoop, this);

  ros::spin();
  robot_control_loop_thread_.join();
  return 0;

}

void RobotControl::loadAndInitPlugins()
{
  loadAndInitPlugin(pm_controlled_device_driver_,"controlled_device_drivers");
  loadAndInitPlugin(pm_device_driver_,"device_drivers");
  loadAndInitPlugin(pm_postprocessor_,"postprocessors");
  loadAndInitPlugin(pm_preprocessor_,"preprocessors");
}

void RobotControl::loadAndInitPlugin(PluginManagerPtr pm, std::string prefix)
{
  tug_plugin_manager::RegularPluginPtr new_plugin;

  ros::NodeHandle node_handle(node_handle_, prefix);
  XmlRpc::XmlRpcValue params;
  node_handle.getParam("", params);

  if (!params.valid())
  {
    ROS_WARN_STREAM("No Plugins given for " << prefix);
    return;
  }

  for (XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it)
  {
    XmlRpc::XmlRpcValue & param = it->second;

    if (!param.hasMember("type"))
      throw std::runtime_error(prefix + "/" + it->first + " has no 'type' parameter");

    new_plugin = pm->loadPlugin(it->first, param["type"]);
    pluginBasePtrCast(new_plugin)->initialize(&robot_hardware_, ros::NodeHandle(node_handle, it->first), "");
  }
}

void RobotControl::runRobotControlLoop()
{
  ros::Rate loop_rate(robot_control_loop_rate_);
  ros::Time last_time = ros::Time::now();

  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    try
    {
      readControlledDevice(current_time, elapsed_time);

      doPreprocessing(current_time, elapsed_time);

      controller_manager_.update(current_time, elapsed_time);

      doPostprocessing(current_time, elapsed_time);

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
    node.init();
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
