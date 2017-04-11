#ifndef _TUG_CFG__STRUCT_H_
#define _TUG_CFG__STRUCT_H_

#include <map>
#include <set>
#include <tug_cfg/collection.h>
#include <tug_cfg/key.h>
#include <tug_cfg/type.h>
#include <vector>

namespace tug_cfg
{
class AbstractStruct : public Collection
{
public:
  class Field
  {
  public:
    Field(const std::string& name_, const std::string& unit_,
          const std::string& description_, bool dynamic_, int level_,
          bool ignored_);
    virtual ~Field() = default;

    std::string name;
    std::string unit;
    std::string description;
    bool dynamic;
    int level;
    bool ignored;
  };



  template <typename T>
  class ScalarField : public Field
  {
  public:
    ScalarField(const std::string& name_, const std::string& unit_,
                const std::string& description_, bool dynamic_, int level_,
                bool ignored_, const T& default_value_, const T& min_,
                const T& max_, const std::set<T>& choices_,
                const std::set<T>& suggestions_)
      : Field(name_, unit_, description_, dynamic_, level_, ignored_),
        default_value(default_value_), min(min_), max(max_), choices(choices_),
        suggestions(suggestions_)
    {
    }

    T default_value;
    T min;
    T max;
    const std::set<T> choices;
    const std::set<T> suggestions;
  };



  template <typename MetaT>
  struct FieldTypes
  {
    typedef Field Type;
  };

  template <typename T>
  struct FieldTypes<Scalar<T>>
  {
    typedef ScalarField<T> Type;
  };



  virtual void accept(Key* key, Visitor& visitor) override;
  virtual void accept(const Key* key, ConstVisitor& visitor) const override;
};



template <>
void ScalarKey<const AbstractStruct::Field*>::format(std::ostream& s) const;



template <typename C>
class Struct : public AbstractStruct
{
public:
  typedef C Value;



  class FieldImplBase
  {
  public:
    virtual ~FieldImplBase() = default;

    virtual void accept(C* instance, const Key* parent_key, Visitor& visitor) const = 0;
    virtual void accept(C* instance, const Key* parent_key, ConstVisitor& visitor) const = 0;
  };



  typedef const std::vector<const FieldImplBase*> Fields;



  template <typename MetaT, typename FieldT = typename FieldTypes<MetaT>::Type>
  class FieldImpl : public FieldImplBase
  {
  public:
    FieldImpl(typename MetaT::Value C::* var, const FieldT& field)
      : var_(var), field_(field)
    {
    }

    virtual void accept(C* instance, const Key* parent_key, Visitor& visitor) const override
    {
      ScalarKey<const Field*> key(parent_key, &field_);
      MetaT(instance->*var_).accept(&key, visitor);
    }

    virtual void accept(C* instance, const Key* parent_key, ConstVisitor& visitor) const override
    {
      ScalarKey<const Field*> key(parent_key, &field_);
      MetaT(instance->*var_).accept(&key, visitor);
    }

  protected:
    typename MetaT::Value C::* var_;
    FieldT field_;
  };



  class Type : public tug_cfg::Type
  {
  public:
    Type(const std::string& name, const Fields& fields)
      : name_(name), fields_(fields)
    {
    }

    virtual std::string getName() const
    {
      return name_;
    }

  protected:
    friend class Struct<C>;

    const std::string name_;
    const Fields fields_;
  };



  explicit Struct(Value& value)
    : value_(&value)
  {
  }

  virtual const Type& getType() const override
  {
    return Value::getDefinition();
  }

  void acceptElements(const Key* parent_key, Visitor& visitor) override
  {
    for (const FieldImplBase* field : Value::getDefinition().fields_)
    {
      field->accept(value_, parent_key, visitor);
    }
  }

  void acceptElements(const Key* parent_key, ConstVisitor& visitor) const override
  {
    for (const FieldImplBase* field : Value::getDefinition().fields_)
    {
      field->accept(value_, parent_key, visitor);
    }
  }

protected:
  Value* value_;
};
}

#endif
