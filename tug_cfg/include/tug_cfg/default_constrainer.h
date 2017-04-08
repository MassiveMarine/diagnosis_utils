#ifndef _TUG_CFG__DEFAULT_CONSTRAINER_H_
#define _TUG_CFG__DEFAULT_CONSTRAINER_H_

#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class DefaultConstrainer : public Visitor
{
public:
  DefaultConstrainer();
  virtual ~DefaultConstrainer();

  virtual void visit(Key& key, Map::Instance& value);
  virtual void visit(Key& key, Scalar<bool>::Instance& value);
  virtual void visit(Key& key, Scalar<double>::Instance& value);
  virtual void visit(Key& key, Scalar<int>::Instance& value);
  virtual void visit(Key& key, Scalar<std::string>::Instance& value);
  virtual void visit(Key& key, Struct::Instance& value);
  virtual void visit(Key& key, Vector::Instance& value);
  virtual void visit(Key& key, Object& value);

protected:
  template <typename T, typename M = T>
  static void enforceMin(const std::string& name, T& value, M min_value)
  {
    if (value < min_value)
    {
      value = min_value;
      // TODO: warn
    }
  }

  template <typename T, typename M = T>
  static void enforceMax(const std::string& name, T& value, M max_value)
  {
    if (value > max_value)
    {
      value = max_value;
      // TODO: warn
    }
  }

  template <typename T, typename C = T>
  static void enforceChoices(const std::string& name, T& value, const std::vector<C>& choices)
  {
    // TODO
  }
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__DEFAULT_CONSTRAINER_H_
