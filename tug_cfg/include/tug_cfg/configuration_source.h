#ifndef _TUG_CFG__CONFIGURATION_SOURCE_H_
#define _TUG_CFG__CONFIGURATION_SOURCE_H_

#include <map>
#include <string>
#include <vector>

namespace tug_cfg
{
class ConfigurationSource
{
  class Key
  {
  public:
    Key(std::size_t index);
    Key(const std::string& name);
    Key(const Key* parent, std::size_t index);
    Key(const Key* parent, const std::string& name);
  };

  ConfigurationSource();
  virtual ~ConfigurationSource();

  virtual void load(const Key& key, bool& result, bool default_value) = 0;
  virtual void load(const Key& key, double& result, double default_value) = 0;
  virtual void load(const Key& key, int& result, int default_value) = 0;
  virtual void load(const Key& key, std::string& result, const std::string& default_value) = 0;

  template <typename T>
  void load(const std::string& name, std::vector<T>& result)
  {
    // TODO
  }

  //virtual void loadVector(const std::string& name)

protected:
};
}

#endif
