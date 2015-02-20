#ifndef PLUGIN_MANAGER_PLUGIN_MANAGER_EXCEPTIONS_H
#define PLUGIN_MANAGER_PLUGIN_MANAGER_EXCEPTIONS_H

#include <stdexcept>

namespace tug_plugin_manager
{

class PluginManagerException: public std::runtime_error
{
public:
  PluginManagerException(const std::string error_desc) :
      std::runtime_error(error_desc)
  {
  }
};

class PluginAlreadyInListException: public PluginManagerException
{
public:
  PluginAlreadyInListException(const std::string error_desc) :
      PluginManagerException(error_desc)
  {
  }
};

class PluginCannotBeCreatedException: public PluginManagerException
{
public:
  PluginCannotBeCreatedException(const std::string error_desc) :
      PluginManagerException(error_desc)
  {
  }
};

}

#endif
