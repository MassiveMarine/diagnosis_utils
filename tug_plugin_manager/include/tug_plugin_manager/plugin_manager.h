#ifndef PLUGIN_MANAGER_PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_PLUGIN_MANAGER_H

#include <vector>
#include <pluginlib/class_loader.h>
#include <ros/console.h>

#include <tug_plugin_manager/plugin_base.h>
#include <tug_plugin_manager/plugin_spec.h>
#include <tug_plugin_manager/plugin_manager_exceptions.h>

namespace tug_plugin_manager
{
    template <class T> class PluginManager
    {
        public:
            PluginManager() :
                    plugin_loader_("tug_plugin_manager", "tug_plugin_manager::RegularPlugin")
            { }

            PluginManager(std::string package, std::string base_class) : plugin_loader_(package, base_class)
            { }

            virtual ~PluginManager()
            { }

            boost::shared_ptr<T> loadPlugin(const std::string& name, const std::string& type)
            {
                ROS_INFO("Will load plugin '%s' of type '%s'", name.c_str(), type.c_str());

                boost::shared_ptr<T> newPluginInstance;

                if (getPluginInstanceByName(name))
                    throw(PluginAlreadyInListException(
                            std::string("The plugin '").append(name.c_str()).append("' was already loaded inside the plugin manager")));

                newPluginInstance = plugin_loader_.createInstance(type);

                if (!newPluginInstance)
                    throw(PluginCannotBeCreatedException(std::string("Could not create object of plugin '").append(name.c_str()).append("'")));

                plugin_list_.push_back(PluginSpec<T>(name, type, newPluginInstance));

                //  ROS_INFO("Now %d plugin(s) is/are loaded!", (int)plugin_list_.size() );

                return newPluginInstance;
            }

            boost::shared_ptr<T> getPluginInstanceByName(const std::string& name)
            {
                for (size_t i = 0; i < plugin_list_.size(); ++i)
                    if (plugin_list_[i].name == name)
                        return plugin_list_[i].instance;

                return boost::shared_ptr<T>();
            }

            const std::vector<std::string> getPluginNames()
            {
                std::vector<std::string> names_list;
                for (size_t i = 0; i < plugin_list_.size(); ++i)
                    names_list.push_back(plugin_list_[i].name);

                return names_list;
            }

            std::vector<PluginSpec<T> > getPluginList()
            {
                return plugin_list_;
            }

        private:
            pluginlib::ClassLoader<T> plugin_loader_;
            std::vector<PluginSpec<T> > plugin_list_;
    };

}

#endif // PLUGIN_MANAGER_H