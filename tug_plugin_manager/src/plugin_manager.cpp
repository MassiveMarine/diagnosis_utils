#include <tug_plugin_manager/plugin_manager.h>
#include <ros/console.h>

//#include <robot_plugins/plugin_base.h>

namespace tug_plugin_manager
{

PluginManager::PluginManager(std::string package_name) :
    plugin_loader_(package_name, "tug_plugin_manager::RegularPlugin")
{
}

PluginManager::~PluginManager()
{
}

bool PluginManager::loadPlugin(const std::string& name, const std::string& type)
{
  ROS_INFO("Will load plugin '%s' of type '%s'", name.c_str(),type.c_str());

  boost::shared_ptr<RegularPlugin> newPluginInstance;

  if (getPluginInstanceByName(name))
    throw(PluginAlreadyInListException(std::string("The plugin '").append(name.c_str()).append("' was already loaded inside the plugin manager")));

  newPluginInstance = plugin_loader_.createInstance(type);

  if (!newPluginInstance)
    throw(PluginCannotBeCreatedException(std::string("Could not create object of plugin '").append(name.c_str()).append("'")));

  newPluginInstance->initialize("BLIBLABLO");

  plugin_list_.resize(plugin_list_.size() + 1);
  plugin_list_[plugin_list_.size() - 1].name = name;
  plugin_list_[plugin_list_.size() - 1].type = type;
  plugin_list_[plugin_list_.size() - 1].instance = newPluginInstance;

//  ROS_INFO("Now %d plugin(s) is/are loaded!", (int)plugin_list_.size() );

  return true;
}

boost::shared_ptr<RegularPlugin> PluginManager::getPluginInstanceByName(const std::string& name)
{
  for (size_t i = 0; i < plugin_list_.size(); ++i)
    if (plugin_list_[i].name == name)
      return plugin_list_[i].instance;

  return boost::shared_ptr<RegularPlugin>();
}

const std::vector<std::string> PluginManager::getPluginNames()
{
  std::vector<std::string> names_list;
  for (size_t i = 0; i < plugin_list_.size(); ++i)
    names_list.push_back(plugin_list_[i].name);

  return names_list;
}
std::vector<PluginSpec> PluginManager::getPluginList()
{
  return plugin_list_;
}

}
