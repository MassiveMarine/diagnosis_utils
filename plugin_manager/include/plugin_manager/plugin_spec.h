#ifndef PLUGIN_SPEC_H
#define PLUGIN_SPEC_H

#include <string>
//#include <robot_plugins/plugin_base.h>

namespace plugin_manager
{

struct PluginSpec
{
  std::string name;
  std::string type;
  boost::shared_ptr<plugin_base::RegularPlugin> instance;
};

}

#endif // PLUGIN_SPEC_H
