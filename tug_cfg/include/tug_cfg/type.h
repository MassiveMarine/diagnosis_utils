#ifndef _TUG_CFG__TYPE_H_
#define _TUG_CFG__TYPE_H_

#include <string>

namespace tug_cfg
{
class Type
{
public:
  Type() = default;
  virtual ~Type() = default;

  virtual std::string getName() const = 0;  // potentially recursive
};
}

#endif
