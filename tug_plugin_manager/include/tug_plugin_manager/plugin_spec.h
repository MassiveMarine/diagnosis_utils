#ifndef PLUGIN_SPEC_H
#define PLUGIN_SPEC_H

#include <string>

namespace tug_plugin_manager
{

struct PluginSpec
{
  std::string name;
  std::string type;
  boost::shared_ptr<RegularPlugin> instance;
};

}

#endif // PLUGIN_SPEC_H
