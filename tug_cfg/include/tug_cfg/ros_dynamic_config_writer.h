#ifndef _TUG_CFG__ROS_DYNAMIC_CONFIG_WRITER_H_
#define _TUG_CFG__ROS_DYNAMIC_CONFIG_WRITER_H_

#include <dynamic_reconfigure/Config.h>
#include <tug_cfg/scalar.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class RosDynamicConfigWriter : public ConstVisitor
{
public:
  RosDynamicConfigWriter(dynamic_reconfigure::Config* config);
  virtual ~RosDynamicConfigWriter();

  virtual void visit(const Key* key, const AbstractMap& value) override;
  virtual void visit(const Key* key, const AbstractSequence& value) override;
  virtual void visit(const Key* key, const AbstractStruct& value) override;
  virtual void visit(const Key* key, const Scalar<bool>& value) override;
  virtual void visit(const Key* key, const Scalar<double>& value) override;
  virtual void visit(const Key* key, const Scalar<int>& value) override;
  virtual void visit(const Key* key, const Scalar<std::string>& value) override;
  virtual void visit(const Key* key, const Object& value) override;

protected:
  template <typename T, typename P>
  void visitScalar(const Key* key, const Scalar<T>& value,
                   std::vector<P>& values);

  dynamic_reconfigure::Config* config_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__ROS_DYNAMIC_CONFIG_WRITER_H_
