#ifndef _TUG_CFG__DEFAULT_CONSTRAINER_H_
#define _TUG_CFG__DEFAULT_CONSTRAINER_H_

#include <set>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class DefaultConstrainer : public Visitor
{
public:
  virtual void visit(Key* key, AbstractMap& value) override;
  virtual void visit(Key* key, Scalar<bool>& value) override;
  virtual void visit(Key* key, Scalar<double>& value) override;
  virtual void visit(Key* key, Scalar<int>& value) override;
  virtual void visit(Key* key, Scalar<std::string>& value) override;
  virtual void visit(Key* key, AbstractStruct& value) override;
  virtual void visit(Key* key, AbstractSequence& value) override;
  virtual void visit(Key* key, Object& value) override;

protected:
  template <typename T>
  static void constrainScalar(Key* key, Scalar<T>& value);

  template <typename T>
  static void enforceMin(Key* key, T& value, const T& min_value);

  template <typename T>
  static void enforceMax(Key* key, T& value, const T& max_value);

  template <typename T>
  static void enforceChoices(Key* key, T& value, const std::set<T>& choices,
                             const T& default_value);
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__DEFAULT_CONSTRAINER_H_
