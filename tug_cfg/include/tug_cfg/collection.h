#ifndef _TUG_CFG__COLLECTION_H_
#define _TUG_CFG__COLLECTION_H_

#include <map>
#include <tug_cfg/key.h>
#include <tug_cfg/object.h>
#include <tug_cfg/type.h>
#include <typeinfo>
#include <vector>

namespace tug_cfg
{
class Collection : public Object
{
public:
  virtual void acceptElements(Visitor& visitor) = 0;
  virtual void acceptElements(ConstVisitor& visitor) const = 0;
};



class ExtensibleCollection : public Collection
{
public:
  virtual void acceptNewElement(Visitor& visitor) = 0;
};



class AbstractSequence : public ExtensibleCollection
{
public:
  virtual void accept(Key& key, Visitor& visitor) override;
  virtual void accept(const Key& key, ConstVisitor& visitor) const override;
};



template <typename ElementMetaT>
class Sequence : public AbstractSequence
{
public:
  typedef std::vector<typename ElementMetaT::Value> Value;

  class Type : public tug_cfg::Type
  {
    virtual std::string getName() const override
    {
      typename ElementMetaT::Value value;
      return ElementMetaT(value).getType().getName() + "[]";
    }
  };

  explicit Sequence(Value& elements)
    : elements_(elements)
  {
  }

  virtual const tug_cfg::Type& getType() const override
  {
    static Type type;
    return type;
  }

  virtual void acceptElements(Visitor& visitor) override
  {
    for (std::size_t i = 0; i < elements_.size(); ++i)
    {
      ScalarKey<int> key(static_cast<int>(i));
      ElementMetaT(elements_[i]).accept(key, visitor);
    }
  }

  virtual void acceptElements(ConstVisitor& visitor) const override
  {
    for (std::size_t i = 0; i < elements_.size(); ++i)
    {
      ScalarKey<int> key(static_cast<int>(i));
      ElementMetaT(elements_[i]).accept(key, visitor);
    }
  }

  virtual void acceptNewElement(Visitor& visitor) override
  {
    ScalarKey<int> key(static_cast<int>(elements_.size()));
    typename Value::value_type value;
    ElementMetaT(value).accept(key, visitor);
    elements_.insert(elements_.begin() + key.key, value);
  }

protected:
  Value& elements_;
};



class AbstractMap : public ExtensibleCollection
{
public:
  virtual void accept(Key& key, Visitor& visitor) override;
  virtual void accept(const Key& key, ConstVisitor& visitor) const override;
};



template <typename Key, typename ElementMetaT>
class Map : public AbstractMap
{
public:
  typedef std::map<Key, typename ElementMetaT::Value> Value;

  class Type : public tug_cfg::Type
  {
  public:
    virtual std::string getName() const override
    {
      typename ElementMetaT::Value value;
      return ElementMetaT(value).getType().getName() + "[" + typeid(Key).name()
          + "]";
    }
  };

  explicit Map(Value& elements)
    : elements_(elements)
  {
  }

  virtual const tug_cfg::Type& getType() const override
  {
    static Type type;
    return type;
  }

  virtual void acceptElements(Visitor& visitor) override
  {
    for (typename Value::value_type& element : elements_)
    {
      ScalarKey<Key> key(element.first);
      ElementMetaT(element.second).accept(key, visitor);
    }
  }

  virtual void acceptElements(ConstVisitor& visitor) const
  {
    for (typename Value::value_type& element : elements_)
    {
      ScalarKey<Key> key(element.first);
      ElementMetaT(element.second).accept(key, visitor);
    }
  }

  virtual void acceptNewElement(Visitor& visitor) override
  {
    Key k;
    ScalarKey<Key> key(k);
    typename Value::mapped_type value;
    ElementMetaT(value).accept(key, visitor);
    elements_[key.key] = value;
  }

protected:
  Value& elements_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__COLLECTION_H_
