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

  virtual void visit(Key& key, typename Map::Instance& value) = 0;
  virtual void visit(Key& key, typename Scalar<bool>::Instance& value) = 0;
  virtual void visit(Key& key, typename Scalar<double>::Instance& value) = 0;
  virtual void visit(Key& key, typename Scalar<int>::Instance& value) = 0;
  virtual void visit(Key& key, typename Scalar<std::string>::Instance& value) = 0;
  virtual void visit(Key& key, typename Struct::Instance& value) = 0;
  virtual void visit(Key& key, typename Vector::Instance& value) = 0;
  virtual void visit(Key& key, Object& value) = 0;
};



class ConstVisitor
{
public:
  ConstVisitor() = default;
  virtual ~ConstVisitor() = default;

  virtual void visit(const Key& key, const typename Map::Instance& value) = 0;
  virtual void visit(const Key& key, const typename Scalar<bool>::Instance& value) = 0;
  virtual void visit(const Key& key, const typename Scalar<double>::Instance& value) = 0;
  virtual void visit(const Key& key, const typename Scalar<int>::Instance& value) = 0;
  virtual void visit(const Key& key, const typename Scalar<std::string>::Instance& value) = 0;
  virtual void visit(const Key& key, const typename Struct::Instance& value) = 0;
  virtual void visit(const Key& key, const typename Vector::Instance& value) = 0;
  virtual void visit(const Key& key, const Object& value) = 0;
};
}

#endif
