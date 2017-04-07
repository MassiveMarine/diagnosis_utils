#ifndef _TUG_CFG__FORWARDS_H_
#define _TUG_CFG__FORWARDS_H_

#include <memory>

namespace tug_cfg
{
  class Configuration;
  typedef std::shared_ptr<Configuration> ConfigurationPtr;
  typedef std::shared_ptr<const Configuration> ConfigurationConstPtr;

  class ConstVisitor;
  class Constrainer;

  class Key;
  class Map;

  class Object;
  typedef std::shared_ptr<Object> ObjectPtr;

  class Struct;
  class Type;
  class Vector;

  class Visitor;
}  // namespace tug_cfg

#endif  // _TUG_CFG__FORWARDS_H_
