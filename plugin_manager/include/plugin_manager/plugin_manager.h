#ifndef PLUGIN_MANAGER_PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_PLUGIN_MANAGER_H

#include <vector>
#include <pluginlib/class_loader.h>

#include <plugin_manager/plugin_base.h>
#include <plugin_manager/plugin_spec.h>
#include <plugin_manager/plugin_manager_exceptions.h>

namespace plugin_manager
{
class PluginManager
{
public:
  PluginManager(std::string package_name);
  virtual ~PluginManager();

  bool loadPlugin(const std::string& name);

  boost::shared_ptr<plugin_base::RegularPlugin> getPluginInstanceByName(const std::string& name);
  void getPluginNames(std::vector<std::string> &names);

private:
  PluginManager();
  std::vector<PluginSpec> plugin_list_;
  pluginlib::ClassLoader<plugin_base::RegularPlugin> plugin_loader_;

};

}

#endif // PLUGIN_MANAGER_H
