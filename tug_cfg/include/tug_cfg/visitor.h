#ifndef _TUG_CFG__VISITOR_H_
#define _TUG_CFG__VISITOR_H_

#include <string>
#include <tug_cfg/forwards.h>

namespace tug_cfg
{
class Visitor
{
public:
  Visitor() = default;
  virtual ~Visitor() = default;

  virtual void visit(Key* key, AbstractMap& value) = 0;
  virtual void visit(Key* key, AbstractSequence& value) = 0;
  virtual void visit(Key* key, AbstractStruct& value) = 0;
  virtual void visit(Key* key, Scalar<bool>& value) = 0;
  virtual void visit(Key* key, Scalar<double>& value) = 0;
  virtual void visit(Key* key, Scalar<int>& value) = 0;
  virtual void visit(Key* key, Scalar<std::string>& value) = 0;
  virtual void visit(Key* key, Object& value) = 0;
};



class ConstVisitor
{
public:
  ConstVisitor() = default;
  virtual ~ConstVisitor() = default;

  virtual void visit(const Key* key, const AbstractMap& value) = 0;
  virtual void visit(const Key* key, const AbstractSequence& value) = 0;
  virtual void visit(const Key* key, const AbstractStruct& value) = 0;
  virtual void visit(const Key* key, const Scalar<bool>& value) = 0;
  virtual void visit(const Key* key, const Scalar<double>& value) = 0;
  virtual void visit(const Key* key, const Scalar<int>& value) = 0;
  virtual void visit(const Key* key, const Scalar<std::string>& value) = 0;
  virtual void visit(const Key* key, const Object& value) = 0;
};
}

#endif
