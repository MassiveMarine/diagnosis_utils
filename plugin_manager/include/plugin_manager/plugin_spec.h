#ifndef PLUGIN_SPEC_H
#define PLUGIN_SPEC_H

#include <string>

namespace plugin_manager
{

struct PluginSpec
{
	int temp_number;
	std::string name;
//  hardware_interface::ControllerInfo info;
//  boost::shared_ptr<controller_interface::ControllerBase> c;
};

}

#endif // PLUGIN_SPEC_H
