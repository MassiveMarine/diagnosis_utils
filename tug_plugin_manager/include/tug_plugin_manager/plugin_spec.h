#ifndef TUG_PLUGIN_MANAGER__PLUGIN_SPEC_H
#define TUG_PLUGIN_MANAGER__PLUGIN_SPEC_H

#include <string>
#include <boost/shared_ptr.hpp>

namespace tug_plugin_manager
{

template <class T>
class PluginSpec
{
public:
  PluginSpec(const std::string& _name, const std::string& _type, boost::shared_ptr<T> _instance)
    : name(_name), type(_type), instance(_instance)
  {
  }

  std::string name;
  std::string type;
  boost::shared_ptr<T> instance;
};

}

#endif // TUG_PLUGIN_MANAGER__PLUGIN_SPEC_H
