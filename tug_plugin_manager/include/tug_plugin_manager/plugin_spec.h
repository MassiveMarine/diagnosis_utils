#ifndef PLUGIN_SPEC_H
#define PLUGIN_SPEC_H

#include <string>

namespace tug_plugin_manager
{

typedef boost::shared_ptr<RegularPlugin> RegularPluginPtr;

struct PluginSpec
{
  std::string name;
  std::string type;
  RegularPluginPtr instance;
};

}

#endif // PLUGIN_SPEC_H
