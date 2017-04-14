#ifndef _TUG_CFG__ROS_DYNAMIC_CONFIG_READER_H_
#define _TUG_CFG__ROS_DYNAMIC_CONFIG_READER_H_

#include <dynamic_reconfigure/Config.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class RosDynamicConfigReader : public Visitor
{
public:
  RosDynamicConfigReader(const dynamic_reconfigure::Config* config);
  virtual ~RosDynamicConfigReader();

  virtual void visit(Key* key, AbstractMap& value) override;
  virtual void visit(Key* key, AbstractSequence& value) override;
  virtual void visit(Key* key, AbstractStruct& value) override;
  virtual void visit(Key* key, Scalar<bool>& value) override;
  virtual void visit(Key* key, Scalar<double>& value) override;
  virtual void visit(Key* key, Scalar<int>& value) override;
  virtual void visit(Key* key, Scalar<std::string>& value) override;
  virtual void visit(Key* key, Object& value) override;

protected:
  template <typename T, typename Values>
  void visitScalar(Key* key, Scalar<T>& value, const Values& values);

  const dynamic_reconfigure::Config* config_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__ROS_DYNAMIC_CONFIG_READER_H_
