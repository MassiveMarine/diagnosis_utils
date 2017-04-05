#ifndef _TUG_CFG__VALUE_H_
#define _TUG_CFG__VALUE_H_

#include <string>

namespace tug_cfg
{

class Value
{
public:
  typedef const Value& Ref;

  Value();
  virtual ~Value();

  virtual explicit operator bool() const = 0;
  virtual explicit operator double() const = 0;
  virtual explicit operator int() const = 0;
  virtual explicit operator std::string() const = 0;

  virtual Ref operator[](std::size_t index) const = 0;
  virtual Ref operator[](const std::string& key) const = 0;

protected:
};

}

#endif
