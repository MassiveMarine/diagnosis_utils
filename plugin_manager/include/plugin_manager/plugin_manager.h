#ifndef PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_H

#include <vector>
#include <pluginlib/class_loader.h>
#include <plugin_manager/plugin_spec.h>

namespace plugin_manager
{

class PluginManager
{
public:
	PluginManager();
	virtual ~PluginManager();


	int loadPlugin(const std::string& name);
	//int unloadPlugin(const std::string &name);

	int getPluginByName(const std::string& name);
	void getPluginNames(std::vector<std::string> &names);

private:
	std::vector<PluginSpec> plugin_list_;

};

}

#endif // PLUGIN_MANAGER_H
