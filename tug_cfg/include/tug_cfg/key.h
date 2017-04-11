#ifndef _TUG_CFG__KEY_H_
#define _TUG_CFG__KEY_H_

#include <ostream>

namespace tug_cfg
{
template <typename T> class ScalarKey;

class Key
{
public:
  explicit Key(const Key* parent);
  virtual ~Key() = default;

  const Key* getParent() const;

  virtual void format(std::ostream& s) const = 0;
  void formatPath(std::ostream& s) const;

  /**
   * \throw std::bad_cast
   */
  template <typename T>
  inline T& as()
  {
    return dynamic_cast<ScalarKey<T>&>(*this);
  }

  /**
   * \throw std::bad_cast
   */
  template <typename T>
  inline const T& as() const
  {
    return dynamic_cast<const ScalarKey<T>&>(*this);
  }

  template <typename T>
  T* asPtr()
  {
    ScalarKey<T>* result = dynamic_cast<ScalarKey<T>*>(this);
    if (result != nullptr)
    {
      return &static_cast<T&>(*result);
    }
    ScalarKey<T*>* result2 = dynamic_cast<ScalarKey<T*>*>(this);
    if (result2 != nullptr)
    {
      return *result2;
    }
    return nullptr;
  }

  template <typename T>
  inline const T* asPtr() const
  {
    const ScalarKey<T>* result = dynamic_cast<const ScalarKey<T>*>(this);
    if (result != nullptr)
    {
      return &static_cast<const T&>(*result);
    }
    const ScalarKey<T*>* result2 = dynamic_cast<const ScalarKey<T*>*>(this);
    if (result2 != nullptr)
    {
      return *result2;
    }
    return nullptr;
  }

  template <typename T>
  inline bool is() const
  {
    return dynamic_cast<const ScalarKey<T>*>(this) != nullptr;
  }

protected:
  const Key* const parent_;
};



template <typename T>
class ScalarKey : public Key
{
public:
  inline ScalarKey(const Key* parent, const T& key)
    : Key(parent), key_(key)
  {
  }

  virtual void format(std::ostream& s) const override
  {
    s << '[' << key_ << ']';
  }

  inline operator T&()
  {
    return key_;
  }

  inline operator const T&() const
  {
    return key_;
  }

protected:
  T key_;
};



std::ostream& operator<<(std::ostream& s, const Key& key);
std::ostream& operator<<(std::ostream& s, const Key* key);
}

#endif
