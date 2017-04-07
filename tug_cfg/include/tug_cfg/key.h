#ifndef _TUG_CFG__KEY_H_
#define _TUG_CFG__KEY_H_

namespace tug_cfg
{
class Key
{
public:
  /**
   * \throw std::bad_cast
   */
  template <typename T>
  inline T& as()
  {
    return dynamic_cast<ScalarKey<T>&>(*this).key;
  }

  /**
   * \throw std::bad_cast
   */
  template <typename T>
  inline const T& as() const
  {
    return dynamic_cast<const ScalarKey<T>&>(*this).key;
  }

  template <typename T>
  inline bool is() const
  {
    return dynamic_cast<const ScalarKey<T>*>(this) != nullptr;
  }

protected:
  Key() = default;
  virtual ~Key() = default;

  Key(const Key&) = delete;
  Key& operator=(const Key&) = delete;
};



template <typename T>
class ScalarKey : public Key
{
public:
  ScalarKey(const T& key_)
    : key(key_)
  {
  }

  T key;
};



template <>
class ScalarKey<void> : public Key
{
};
}

#endif
