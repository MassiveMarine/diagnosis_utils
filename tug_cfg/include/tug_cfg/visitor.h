#ifndef _TUG_CFG__VISITOR_H_
#define _TUG_CFG__VISITOR_H_

#include <string>
#include <tug_cfg/forwards.h>
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



class ConstVisitor
{
public:
  ConstVisitor() = default;
  virtual ~ConstVisitor() = default;

  virtual void visit(const Key& key, const Map::Instance& value) = 0;
  virtual void visit(const Key& key, const Scalar<bool>::Instance& value) = 0;
  virtual void visit(const Key& key, const Scalar<double>::Instance& value) = 0;
  virtual void visit(const Key& key, const Scalar<int>::Instance& value) = 0;
  virtual void visit(const Key& key, const Scalar<std::string>::Instance& value) = 0;
  virtual void visit(const Key& key, const Struct::Instance& value) = 0;
  virtual void visit(const Key& key, const Vector::Instance& value) = 0;
  virtual void visit(const Key& key, const Object& value) = 0;
};
}

#endif
