#ifndef _TUG_CFG__CONFIGURATION_H_
#define _TUG_CFG__CONFIGURATION_H_

#include <tug_cfg/configuration_sink.h>
#include <tug_cfg/configuration_source.h>

namespace tug_cfg
{

class Configuration
{
public:
  Configuration();
  virtual ~Configuration();

  virtual void load(tug_cfg::ConfigurationSource& s) = 0;
  virtual void store(tug_cfg::ConfigurationSink& s) = 0;
  virtual void enforceConstraints() = 0;

protected:
  template <typename T, typename M = T>
  static void enforceMin(const std::string& name, T& value, M min_value)
  {
    if (value < min_value)
    {
      value = min_value;
      // TODO: warn
    }
  }

  template <typename T, typename M = T>
  static void enforceMax(const std::string& name, T& value, M max_value)
  {
    if (value > max_value)
    {
      value = max_value;
      // TODO: warn
    }
  }

  template <typename T, typename C = T>
  static void enforceChoices(const std::string& name, T& value, const std::vector<C>& choices)
  {
    // TODO
  }
};

}

#endif
