#ifndef _TUG_CFG__OBJECT_H_
#define _TUG_CFG__OBJECT_H_

#include <tug_cfg/forwards.h>

namespace tug_cfg
{
class Object
{
public:
  virtual ~Object() = default;

  virtual void accept(Key& key, Visitor& visitor) = 0;
  virtual void accept(const Key& key, ConstVisitor& visitor) const = 0;
  virtual const Type& getType() const = 0;
};
}

#endif  // _TUG_CFG__OBJECT_H_
