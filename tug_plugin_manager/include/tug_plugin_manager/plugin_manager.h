#ifndef PLUGIN_MANAGER_PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_PLUGIN_MANAGER_H

#include <vector>
#include <pluginlib/class_loader.h>

#include <tug_plugin_manager/plugin_base.h>
#include <tug_plugin_manager/plugin_spec.h>
#include <tug_plugin_manager/plugin_manager_exceptions.h>

namespace tug_plugin_manager
{
class PluginManager
{
public:
  PluginManager(std::string package_name);
  virtual ~PluginManager();

  bool loadPlugin(const std::string& name, const std::string& type);

  boost::shared_ptr<RegularPlugin> getPluginInstanceByName(const std::string& name);
  const std::vector<std::string> getPluginNames();
  std::vector<PluginSpec> getPluginList();

private:
  PluginManager();
  std::vector<PluginSpec> plugin_list_;
  pluginlib::ClassLoader<RegularPlugin> plugin_loader_;

};

}

#endif // PLUGIN_MANAGER_H
