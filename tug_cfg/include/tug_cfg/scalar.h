#ifndef _TUG_CFG__SCALAR_H_
#define _TUG_CFG__SCALAR_H_

#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
template <typename T>
class Scalar : public Type
{
public:
  class Instance : public Object
  {
  public:
    inline Instance(T& value)
      : value_(value)
    {
    }

    inline operator T&()
    {
      return value_;
    }

    inline operator const T&() const
    {
      return value_;
    }

    virtual void accept(Key& key, Visitor& visitor)
    {
      visitor.visit(key, *this);
    }

    virtual void accept(const Key& key, ConstVisitor& visitor) const
    {
      visitor.visit(key, *this);
    }

    virtual const Type& getType()
    {
      static Scalar<T> type;
      return type;
    }

  protected:
    T& value_;
  };

  virtual std::string getName() const
  {
    return typeid(T).name();
  }
};
}

#endif
