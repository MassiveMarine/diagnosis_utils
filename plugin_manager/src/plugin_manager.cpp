#include <plugin_manager/plugin_manager.h>
#include <ros/console.h>

#include <robot_plugins/plugin_base.h>
//#include <robot_plugins/first_robot.h>

namespace plugin_manager
{

PluginManager::PluginManager()
{
	ROS_WARN("PluginManager::created");
}

PluginManager::~PluginManager()
{}

int PluginManager::loadPlugin(const std::string& name)
{
	ROS_WARN("Will load plugin '%s'", name.c_str());


	pluginlib::ClassLoader<plugin_base::RegularPlugin> robot_loader("robot_plugins", "plugin_base::RegularPlugin");

	try
	{
		boost::shared_ptr<plugin_base::RegularPlugin> firstRobot = robot_loader.createInstance("plugin_robot_ns::FirstRobotLoigge");
		firstRobot->initialize("BLIBLABLO");

		ROS_INFO_STREAM("Name of first robot: " << firstRobot->getName());
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}

	ROS_WARN("Should load plugin '%s'", name.c_str());



	// Checks that we're not duplicating controllers
	for (size_t j = 0; j < plugin_list_.size(); ++j)
	{
		if (plugin_list_[j].name == name)
		{
			ROS_ERROR("A plugin named '%s' was already loaded inside the plugin manager", name.c_str());
			return -1;
		}
	}

	// Constructs the controller
	// ToDo

	// checks if controller was constructed
	// ToDo

	// Initializes the controller
	// ToDo

	// Adds the controller to the new list
	plugin_list_.resize(plugin_list_.size() + 1);
//	plugin_list_[plugin_list_.size() - 1].info.type = type;
//	plugin_list_[plugin_list_.size() - 1].info.hardware_interface = c->getHardwareInterfaceType();
	plugin_list_[plugin_list_.size() - 1].name = name;
	plugin_list_[plugin_list_.size() - 1].temp_number = plugin_list_.size();

//	plugin_list_[plugin_list_.size() - 1].info.resources = claimed_resources;
//	plugin_list_[plugin_list_.size() - 1].c = c;

	ROS_WARN("Successfully load plugin '%s'", name.c_str());
	return -1;
}

//bool PluginManager::unloadPlugin(const std::string &name)
//{
//	ROS_ERROR("PluginManager::unloadPlugin");
//	ROS_ERROR_STREAM("With name: " << name);
//	return false;
//}

int PluginManager::getPluginByName(const std::string& name)
{
  for (size_t i = 0; i < plugin_list_.size(); ++i)
  {
    if (plugin_list_[i].name == name)
      return plugin_list_[i].temp_number;
  }
  return -1;
}

void PluginManager::getPluginNames(std::vector<std::string> &names)
{
  names.clear();
  for (size_t i = 0; i < plugin_list_.size(); ++i)
  {
    names.push_back(plugin_list_[i].name);
  }
}

}
