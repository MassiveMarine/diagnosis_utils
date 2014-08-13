#ifndef ROBOT_PLUGINS_PLUGIN_BASE_H_
#define ROBOT_PLUGINS_PLUGIN_BASE_H_

#include <string.h>

namespace plugin_base
{
class RegularPlugin
{
public:
  virtual void initialize(std::string name) = 0;
  virtual std::string getName() = 0;
  virtual ~RegularPlugin()
  {
  }

protected:
  RegularPlugin()
  {
  }
};
}
;
#endif
