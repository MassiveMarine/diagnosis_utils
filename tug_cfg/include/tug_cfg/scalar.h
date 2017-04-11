#ifndef _TUG_CFG__SCALAR_H_
#define _TUG_CFG__SCALAR_H_

#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <tug_cfg/visitor.h>
#include <typeinfo>

namespace tug_cfg
{
class AbstractScalar : public Object
{
};

template <typename T>
class Scalar : public AbstractScalar
{
public:
  typedef T Value;

  class Type : public tug_cfg::Type
  {
  public:
    virtual std::string getName() const override
    {
      return typeid(Value).name();
    }
  };

  inline explicit Scalar(Value& value)
    : value_(value)
  {
  }

  inline operator Value&()
  {
    return value_;
  }

  inline operator const Value&() const
  {
    return value_;
  }

  inline Scalar<T>& operator=(const T& value)
  {
    value_ = value;
    return *this;
  }

  virtual void accept(Key* key, Visitor& visitor) override
  {
    visitor.visit(key, *this);
  }

  virtual void accept(const Key* key, ConstVisitor& visitor) const override
  {
    visitor.visit(key, *this);
  }

  virtual const Type& getType() const override
  {
    static Type type;
    return type;
  }

protected:
  Value& value_;
};
}

#endif
