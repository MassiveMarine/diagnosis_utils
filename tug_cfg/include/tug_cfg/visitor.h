#ifndef _TUG_CFG__VISITOR_H_
#define _TUG_CFG__VISITOR_H_

#include <string>
#include <tug_cfg/key.h>
#include <tug_cfg/scalar.h>

namespace tug_cfg
{
class Visitor
{
public:
  Visitor() = default;
  virtual ~Visitor() = default;

  virtual void visit(Key& key, Map::Instance& value) = 0;
  virtual void visit(Key& key, Scalar<bool>::Instance& value) = 0;
  virtual void visit(Key& key, Scalar<double>::Instance& value) = 0;
  virtual void visit(Key& key, Scalar<int>::Instance& value) = 0;
  virtual void visit(Key& key, Scalar<std::string>::Instance& value) = 0;
  virtual void visit(Key& key, Struct::Instance& value) = 0;
  virtual void visit(Key& key, Vector::Instance& value) = 0;
  virtual void visit(Key& key, Object& value) = 0;
};
}

#endif
