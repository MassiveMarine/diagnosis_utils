#include <plugin_manager/plugin_manager.h>
#include <ros/console.h>

//#include <robot_plugins/plugin_base.h>

namespace plugin_manager
{

PluginManager::PluginManager(std::string package_name) :
    plugin_loader_(package_name, "plugin_base::RegularPlugin")
{
}

PluginManager::~PluginManager()
{
}

bool PluginManager::loadPlugin(const std::string& type, const std::string& name)
{
  ROS_WARN("Will load plugin '%s'", type.c_str());

  boost::shared_ptr<plugin_base::RegularPlugin> newPluginInstance;

  try
  {
    if (getPluginInstanceByName(type))
      throw(plugin_manager::PluginAlreadyInListException(type.c_str()));

    newPluginInstance = plugin_loader_.createInstance(type);

    if (!newPluginInstance)
      throw(plugin_manager::PluginCannotBeCreatedException(type.c_str()));

    newPluginInstance->initialize("BLIBLABLO");

    plugin_list_.resize(plugin_list_.size() + 1);
    plugin_list_[plugin_list_.size() - 1].name = name;
    plugin_list_[plugin_list_.size() - 1].type = type;
    plugin_list_[plugin_list_.size() - 1].instance = newPluginInstance;

  }
  catch (plugin_manager::PluginAlreadyInListException& ex)
  {
    ROS_ERROR_STREAM("A plugin '" << ex.what() << "' was already loaded inside the plugin manager");
    return false;
  }
  catch (plugin_manager::PluginCannotBeCreatedException& ex)
  {
    ROS_ERROR_STREAM("Could not create object of plugin '" << ex.what() << "'");
    return false;
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    return false;
  }
  return true;
}

boost::shared_ptr<plugin_base::RegularPlugin> PluginManager::getPluginInstanceByName(const std::string& name)
{
  for (size_t i = 0; i < plugin_list_.size(); ++i)
    if (plugin_list_[i].name == name)
      return plugin_list_[i].instance;

  return boost::shared_ptr<plugin_base::RegularPlugin>();
}

void PluginManager::getPluginNames(std::vector<std::string> &names)
{
  names.clear();
  for (size_t i = 0; i < plugin_list_.size(); ++i)
    names.push_back(plugin_list_[i].name);
}

}
