#ifndef PLUGIN_SPEC_H
#define PLUGIN_SPEC_H

#include <string>

namespace tug_plugin_manager
{

    template <class T>
    struct PluginSpec
    {
        PluginSpec(std::string _name, std::string _type, boost::shared_ptr<T> _instance) : name(_name), type(_type), instance(_instance)
        { }
      std::string name;
      std::string type;
      boost::shared_ptr<T> instance;
    };

}

#endif // PLUGIN_SPEC_H
