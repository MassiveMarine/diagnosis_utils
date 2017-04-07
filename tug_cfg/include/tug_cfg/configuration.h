#ifndef _TUG_CFG__CONFIGURATION_H_
#define _TUG_CFG__CONFIGURATION_H_

#include <tug_cfg/forwards.h>

namespace tug_cfg
{
class Configuration
{
public:
  Configuration();
  virtual ~Configuration();

  virtual void load(Visitor& loader, Visitor& constrainer = DefaultConstrainer());
  virtual void store(ConstVisitor& storer) const;

  virtual void accept(Visitor& s) = 0;
  virtual void accept(ConstVisitor& s) const = 0;

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
