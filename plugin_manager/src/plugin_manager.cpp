#include <plugin_manager/plugin_manager.h>
#include <ros/console.h>

//#include <robot_plugins/plugin_base.h>

namespace plugin_manager
{

PluginManager::PluginManager(std::string package_name) :
		plugin_loader_(package_name, "plugin_base::RegularPlugin")
{
	ROS_WARN("PluginManager::created");
}

PluginManager::~PluginManager()
{}

int PluginManager::loadPlugin(const std::string& name)
{
	ROS_WARN("Will load plugin '%s'", name.c_str());

	// Checks that we're not duplicating controllers
	for (size_t j = 0; j < plugin_list_.size(); ++j)
		if (plugin_list_[j].name == name)
		{
			ROS_ERROR("A plugin named '%s' was already loaded inside the plugin manager", name.c_str());
			return -1;
		}

	boost::shared_ptr<plugin_base::RegularPlugin> newPluginInstance;

	try
	{
		// Constructs the plugin_class
		newPluginInstance = plugin_loader_.createInstance(name);

		// checks if plugin_class was constructed
		if (!newPluginInstance)
		{
			ROS_ERROR("Could not load plugin '%s' because plugin type '%s' does not exist.",  name.c_str(), "ToDo");
			ROS_ERROR("Use 'rosservice call controller_manager/list_controller_types' to get the available types");
			return -1;
		}


		// Initializes the plugin_class
		newPluginInstance->initialize("BLIBLABLO");

		// Adds the plugin_class to the new list
		plugin_list_.resize(plugin_list_.size() + 1);

		plugin_list_[plugin_list_.size() - 1].name = name;
		plugin_list_[plugin_list_.size() - 1].id = plugin_list_.size();
		plugin_list_[plugin_list_.size() - 1].instance = newPluginInstance;

	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
		return -1;
	}
	return plugin_list_[plugin_list_.size() - 1].id;
}

//bool PluginManager::unloadPlugin(const std::string &name)
//{
//	ROS_ERROR("PluginManager::unloadPlugin");
//	ROS_ERROR_STREAM("With name: " << name);
//	return false;
//}

boost::shared_ptr<plugin_base::RegularPlugin> PluginManager::getPluginInstanceByName(const std::string& name)
{
  for (size_t i = 0; i < plugin_list_.size(); ++i)
  {
    if (plugin_list_[i].name == name)
      return plugin_list_[i].instance;
  }
  return boost::shared_ptr<plugin_base::RegularPlugin>();
}

void PluginManager::getPluginNames(std::vector<std::string> &names)
{
  names.clear();
  for (size_t i = 0; i < plugin_list_.size(); ++i)
    names.push_back(plugin_list_[i].name);
}

}
