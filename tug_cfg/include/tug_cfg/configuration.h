#ifndef _TUG_CFG__CONFIGURATION_H_
#define _TUG_CFG__CONFIGURATION_H_

#include <tug_cfg/forwards.h>
#include <tug_cfg/struct.h>
#include <tug_cfg/key.h>

namespace tug_cfg
{
class Configuration
{
public:
  Configuration() = default;
  virtual ~Configuration() = default;

  virtual void constrain();
  virtual void constrain(Visitor& constrainer) = 0;
  virtual void load(Visitor& source);
  virtual void load(Visitor& source, Visitor& constrainer) = 0;
  virtual void store(ConstVisitor& sink) const = 0;
};



template <typename C>
class ConfigurationImpl : public Configuration, public StructImpl<C>::Instance
{
  typedef StructImpl<C> TypeImpl;

  ConfigurationImpl()
    : TypeImpl(*this)
  {
  }

  virtual void constrain(Visitor& constrainer) override
  {
    // We should definitely use accept(void, visitor) here because else, visitor
    // doesn't know that we are accepting a struct, and cannot check for
    // superfluous parameters:
    accept(ScalarKey<void>(), constrainer);
  }

  virtual void load(Visitor& source, Visitor& constrainer) override
  {
    accept(ScalarKey<void>(), source);
    accept(ScalarKey<void>(), constrainer);
  }

  virtual void store(ConstVisitor& sink) const override
  {
    accept(ScalarKey<void>(), sink);
  }
};
}

#endif
