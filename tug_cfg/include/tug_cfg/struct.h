#ifndef _TUG_CFG__STRUCT_H_
#define _TUG_CFG__STRUCT_H_

#include <map>
#include <tug_cfg/key.h>
#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <tug_cfg/visitor.h>
#include <vector>

namespace tug_cfg
{
class AbstractStruct : public Object
{
public:
  class FieldInfo
  {
  public:
    FieldInfo(const std::string& unit_, const std::string& description_,
              bool dynamic_, int level_, bool ignored_);

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



  virtual void accept(Key& key, Visitor& visitor) override;
  virtual void accept(const Key& key, ConstVisitor& visitor) const override;

  virtual void acceptItems(Visitor& visitor) = 0;
  virtual void acceptItems(ConstVisitor& visitor) const = 0;
};



template <typename C>
class Struct : public AbstractStruct
{
public:
  class FieldImplBase : public Field
  {
  public:
    FieldImplBase(const std::string& key)
      : Field(key)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) = 0;
    virtual void accept(const C& instance, ConstVisitor& visitor) const = 0;
  };



  typedef const std::vector<FieldImplBase*> Fields;



  template <typename T, typename TypeT, typename FieldInfoT>
  class FieldImpl : public FieldImplBase
  {
  public:
    FieldImpl(const std::string& key, const FieldInfoT& info, T C::* var)
      : FieldImplBase(key), info_(info), var_(var)
    {
    }

    virtual void accept(C& instance, Visitor& visitor) override
    {
      TypeT value(instance.*var_);
      visitor.visit(*this, value);
    }

    virtual void accept(const C& instance, ConstVisitor& visitor) const override
    {
      const TypeT value(const_cast<T&>(instance.*var_));
      visitor.visit(*this, value);
    }

    virtual const FieldInfoT& getInfo() const override
    {
      return info_;
    }

  protected:
    FieldInfoT info_;
    T C::* var_;
  };



  template <typename T>
  struct ScalarField
  {
    typedef FieldImpl<T, Scalar<T>, ScalarFieldInfo<T>> Type;
  };

  template <typename T>
  struct VectorField
  {
    typedef FieldImpl<T, Vector<typename T::value_type>, FieldInfo> Type;
  };

  template <typename T>
  struct MapField
  {
    typedef FieldImpl<T, Map<typename T::key_type, typename T::mapped_type>, FieldInfo> Type;
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



  void acceptItems(Visitor& visitor) override
  {
    const Fields& fields(static_cast<const Type&>(getType()).fields_);
    for (typename Fields::const_iterator it = fields.begin();
         it != fields.end(); ++it)
    {
      (*it)->accept(static_cast<C&>(*this), visitor);
    }
  }

  void acceptItems(ConstVisitor& visitor) const override
  {
    const Fields& fields(static_cast<const Type&>(getType()).fields_);
    for (typename Fields::const_iterator it = fields.begin();
         it != fields.end(); ++it)
    {
      (*it)->accept(static_cast<const C&>(*this), visitor);
    }
  }
};
}

#endif
