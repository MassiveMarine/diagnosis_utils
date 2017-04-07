#ifndef _TUG_CFG__STRUCT_H_
#define _TUG_CFG__STRUCT_H_

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
    FieldInfo(const Type& type_, const std::string& unit_,
              const std::string& description_, bool dynamic_, int level_,
              bool ignored_)
      : type(type_), unit(unit_), description(description_), dynamic(dynamic_),
        level(level_), ignored(ignored_)
    {
    }

    Type& type;
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
    ScalarFieldInfo(const T& default_value_, const T& min_, const T& max_,
                    const std::vector<T>& choices_,
                    const std::vector<T>& suggestions_)
      : default_value(default_value_), min(min_), max(max_), choices(choices_),
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
  class Field;
  typedef std::vector<const Field&> Fields;

  class Instance : public Struct::Instance
  {
  public:
    Instance(const StructImpl<C>& type, C& instance)
      : type_(type), instance_(instance)
    {
    }

    virtual void accept(Key& key, Visitor& visitor)
    {
      visitor.visit(key, *this);
    }

    virtual void accept(const Key& key, ConstVisitor& visitor) const
    {
      visitor.visit(key, *this);
    }

    virtual void acceptItems(Visitor& visitor)
    {
      Fields::const_iterator it;
      for (it = type_.fields_.begin(); it != type_.fields_.end(); ++it)
      {
        it->accept(instance_, visitor);
      }
    }

    virtual void acceptItems(ConstVisitor& visitor) const
    {
      Fields::const_iterator it;
      for (it = type_.fields_.begin(); it != type_.fields_.end(); ++it)
      {
        it->accept(instance_, visitor);
      }
    }

    virtual const StructImpl<C>& getType() const;

  protected:
    const StructImpl<C>& type_;
    C& instance_;
  };



  class Field : public Struct::Field
  {
  public:
    Field(const std::string& key_, const FieldInfo& info_)
      : Struct::Field(key_, info_)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) const = 0;
    virtual void accept(const C& instance, ConstVisitor& visitor) const = 0;
  };



  template <typename T, typename Type>
  class FieldImpl : public Field
  {
  public:
    FieldImpl(const std::string& key_, const FieldInfo& info_, T C::* var_)
      : Struct::Field(key_, info_), var(var_)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) const
    {
      visitor.visit(*this, Type::Instance(instance.*var));
    }

    virtual void accept(const C& instance, ConstVisitor& visitor) const
    {
      visitor.visit(*this, Type::Instance(instance.*var));
    }

    T C::* const var;
  };



  template <typename T>
  class ScalarField : public Struct::ScalarField<T>
  {
    ScalarField(const Type& type, const std::string& unit,
                const std::string& description, bool dynamic, int level,
                bool ignored, const T& default_value, const T& min,
                const T& max, const std::vector<T>& choices,
                const std::vector<T>& suggestions, T C::* var)
      : Struct::ScalarField(type, unit, description, dynamic, level, ignored,
                            default_value, min, max, choices, suggestions),
        var_(var)
    {
    }

  protected:
    T C::* var_;
  };



  StructImpl(const std::string& name, const std::vector<const Field&>& fields)
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
