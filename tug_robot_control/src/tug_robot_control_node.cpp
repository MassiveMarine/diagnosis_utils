#include <tug_robot_control/tug_robot_control_node.h>

namespace tug_robot_control
{
RobotControl::RobotControl() :
    node_handle_("~"),
    robot_hardware_(),
    controller_manager_(&robot_hardware_,ros::NodeHandle(node_handle_, "controllers")),
    pm_controlled_device_driver_(new PluginManager<RegularPlugin>()),
    pm_device_driver_(new PluginManager<RegularPlugin>()),
    pm_preprocessor_(new PluginManager<RegularPlugin>()),
    pm_postprocessor_(new PluginManager<RegularPlugin>())
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
  RegularPluginPtr new_plugin;

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
  ROS_DEBUG("run robot control loop");
  ros::Rate loop_rate(robot_control_loop_rate_);
  ros::Time last_time = ros::Time::now();

  ROS_DEBUG("before while");
  while (ros::ok())
  {
   ROS_DEBUG("get current time and elapsed time");
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    try
    {
      ROS_DEBUG("read controlled devices");
      readControlledDevice(current_time, elapsed_time);
      ROS_DEBUG("perform preprocessing");
      doPreprocessing(current_time, elapsed_time);
      ROS_DEBUG("update controller");
      controller_manager_.update(current_time, elapsed_time);
      ROS_DEBUG("perform postprocessing");
      doPostprocessing(current_time, elapsed_time);
      ROS_DEBUG("write controlled devices");
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
  ROS_DEBUG("read controolled devices called");
  std::vector<PluginSpecInst> list = pm_controlled_device_driver_->getPluginList();
  ROS_DEBUG_STREAM("got list with size " << list.size());
  for (unsigned int i = 0; i < list.size(); i++)
  {
    ROS_DEBUG_STREAM("itterate through list at position " << i);
    ControlledDeviceDriverPtr driver = controlledDeviceDriverPtrCast(list.at(i).instance);
    ROS_DEBUG("casted drive");
    if (driver)
      driver->read(time, period);
  }
}
void RobotControl::writeControlledDevice(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpecInst> list = pm_controlled_device_driver_->getPluginList();
  for (unsigned int i = 0; i < list.size(); i++)
  {
    ControlledDeviceDriverPtr driver = controlledDeviceDriverPtrCast(list.at(i).instance);
    if (driver)
      driver->write(time, period);
  }
}

void RobotControl::doPreprocessing(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpecInst> postprocessor_list = pm_postprocessor_->getPluginList();
  for (unsigned int i = 0; i < postprocessor_list.size(); i++)
  {
    PostprocessorPtr processor = postprocessorPtrCast(postprocessor_list.at(i).instance);
    if (processor)
      processor->process(time, period);
  }
}
void RobotControl::doPostprocessing(const ros::Time& time, const ros::Duration& period)
{
  std::vector<PluginSpecInst> preprocessor_list = pm_preprocessor_->getPluginList();
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
