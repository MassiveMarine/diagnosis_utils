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
  class FieldInfo
  {
  public:
    FieldInfo(const std::string& unit_, const std::string& description_,
              bool dynamic_, int level_, bool ignored_);
    virtual ~FieldInfo() = default;

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
                    const std::set<T>& choices_,
                    const std::set<T>& suggestions_)
      : FieldInfo(unit_, description_, dynamic_, level_, ignored_),
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
  struct FieldInfoTypes
  {
    typedef FieldInfo Type;
  };

  template <typename T>
  struct FieldInfoTypes<Scalar<T>>
  {
    typedef ScalarFieldInfo<T> Type;
  };



  class Field : public Key
  {
  public:
    explicit Field(const std::string& key_)
      : key(key_)
    {
    }

    virtual const FieldInfo& getInfo() const = 0;

    const std::string key;
  };



  virtual void accept(Key& key, Visitor& visitor) override;
  virtual void accept(const Key& key, ConstVisitor& visitor) const override;
};



template <typename C>
class Struct : public AbstractStruct
{
public:
  typedef C Value;



  class FieldImplBase : public Field
  {
  public:
    explicit FieldImplBase(const std::string& key)
      : Field(key)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) = 0;
    virtual void accept(const C& instance, ConstVisitor& visitor) const = 0;
  };



  typedef const std::vector<FieldImplBase*> Fields;



  template <typename MetaT, typename FieldInfoT = typename FieldInfoTypes<MetaT>::Type>
  class FieldImpl : public FieldImplBase
  {
  public:
    FieldImpl(const std::string& key, const FieldInfoT& info,
              typename MetaT::Value C::* var)
      : FieldImplBase(key), info_(info), var_(var)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) override
    {
      MetaT(instance.*var_).accept(*this, visitor);
    }

    virtual void accept(const C& instance, ConstVisitor& visitor) const override
    {
      MetaT(const_cast<C&>(instance).*var_).accept(*this, visitor);
    }

    virtual const FieldInfoT& getInfo() const override
    {
      return info_;
    }

  protected:
    FieldInfoT info_;
    typename MetaT::Value C::* var_;
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
    : value_(value)
  {
  }

  virtual const Type& getType() const override
  {
    return Value::getDefinition();
  }

  void acceptElements(Visitor& visitor) override
  {
    for (FieldImplBase* field : Value::getDefinition().fields_)
    {
      field->accept(value_, visitor);
    }
  }

  void acceptElements(ConstVisitor& visitor) const override
  {
    const Fields& fields(static_cast<const Type&>(getType()).fields_);
    for (typename Fields::const_iterator it = fields.begin();
         it != fields.end(); ++it)
    {
      (*it)->accept(value_, visitor);
    }
  }

protected:
  Value& value_;
};
}

#endif
