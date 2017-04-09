#ifndef _TUG_CFG__STRUCT_H_
#define _TUG_CFG__STRUCT_H_

#include <map>
#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <tug_cfg/visitor.h>
#include <vector>

namespace tug_cfg
{
class Struct : public Type
{
public:
  class Instance : public Object
  {
    virtual void acceptItems(Visitor&) = 0;
    virtual void acceptItems(ConstVisitor&) const = 0;
  };



  class FieldInfo
  {
  public:
    FieldInfo(const std::string& unit_, const std::string& description_,
              bool dynamic_, int level_, bool ignored_)
      : unit(unit_), description(description_), dynamic(dynamic_),
        level(level_), ignored(ignored_)
    {
    }

    std::string unit;
    std::string description;
    bool dynamic;
    int level;
    bool ignored;
  };



  template <typename T>
  class ScalarFieldInfo : public FieldInfo
  {
  public:
    ScalarFieldInfo(const std::string& unit_, const std::string& description_,
                    bool dynamic_, int level_, bool ignored_,
                    const T& default_value_, const T& min_, const T& max_,
                    const std::vector<T>& choices_,
                    const std::vector<T>& suggestions_)
      : FieldInfo(unit_, description_, dynamic_, level_, ignored_),
        default_value(default_value_), min(min_), max(max_), choices(choices_),
        suggestions(suggestions_)
    {
    }

    T default_value;
    T min;
    T max;
    const std::vector<T> choices;
    const std::vector<T> suggestions;
  };



  class Field : public Key
  {
  public:
    Field(const std::string& key_)
      : key(key_)
    {
    }

    virtual const FieldInfo& getInfo() const = 0;

    const std::string key;
  };
};



template <typename C>
class StructImpl : public Struct
{
public:
  class FieldImplBase;
  typedef std::vector<const FieldImplBase&> Fields;



  class Instance : public Struct::Instance
  {
  public:
    typedef StructImpl<C> TypeImpl;

    Instance(C& instance)
      : instance_(instance)
    {
    }

    virtual void accept(Key& key, Visitor& visitor) override
    {
      visitor.visit(key, *this);
    }

    virtual void accept(const Key& key, ConstVisitor& visitor) const override
    {
      visitor.visit(key, *this);
    }

    virtual void acceptItems(Visitor& visitor) override
    {
      Fields::const_iterator it;
      const TypeImpl& type(C::getType());
      for (it = type.fields_.begin(); it != type.fields_.end(); ++it)
      {
        it->accept(instance_, visitor);
      }
    }

    virtual void acceptItems(ConstVisitor& visitor) const override
    {
      Fields::const_iterator it;
      const TypeImpl& type(C::getType());
      for (it = type.fields_.begin(); it != type.fields_.end(); ++it)
      {
        it->accept(instance_, visitor);
      }
    }

    virtual const TypeImpl& getType() const override
    {
      return C::getType();
    }

  protected:
    C& instance_;
  };



  class FieldImplBase : public Struct::Field
  {
  public:
    FieldImplBase(const std::string& key)
      : Struct::Field(key)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) const = 0;
    virtual void accept(const C& instance, ConstVisitor& visitor) const = 0;
  };



  template <typename T, typename TypeT, typename FieldInfoT>
  class FieldImpl : public FieldImplBase
  {
  public:
    FieldImpl(const std::string& key, const FieldInfoT& info, T C::* var)
      : FieldImplBase(key), info_(info), var_(var)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) const override
    {
      visitor.visit(*this, typename TypeT::Instance(instance.*var));
    }

    virtual void accept(const C& instance, ConstVisitor& visitor) const override
    {
      visitor.visit(*this, typename TypeT::Instance(instance.*var));
    }

    virtual const FieldInfoT& getInfo() const override
    {
      return info_;
    }

  protected:
    FieldInfoT info_;
    T C::* const var_;
  };



  template <typename T>
  struct FieldTypes
  {
    typedef FieldImpl<T, Scalar<T>, ScalarFieldInfo<T>> ScalarField;
    typedef FieldImpl<T, Vector, FieldInfo> VectorField;
    typedef FieldImpl<T, Map, FieldInfo> MapField;
  };




  StructImpl(const std::string& name, const Fields& fields)
    : name_(name), fields_(fields)
  {
  }

  virtual std::string getName() const
  {
    return name_;
  }

protected:
  const std::string name_;
  const Fields fields_;
};
}

#endif
