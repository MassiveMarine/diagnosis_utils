#ifndef _TUG_CFG__ROS_DYNAMIC_CONFIG_DESCRIBER_H_
#define _TUG_CFG__ROS_DYNAMIC_CONFIG_DESCRIBER_H_

#include <dynamic_reconfigure/ConfigDescription.h>
#include <tug_cfg/struct.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class RosDynamicConfigDescriber : public ConstVisitor
{
public:
  RosDynamicConfigDescriber(dynamic_reconfigure::ConfigDescription* description);
  virtual ~RosDynamicConfigDescriber();

  virtual void visit(const Key* key, const AbstractMap& value) override;
  virtual void visit(const Key* key, const AbstractSequence& value) override;
  virtual void visit(const Key* key, const AbstractStruct& value) override;
  virtual void visit(const Key* key, const Scalar<bool>& value) override;
  virtual void visit(const Key* key, const Scalar<double>& value) override;
  virtual void visit(const Key* key, const Scalar<int>& value) override;
  virtual void visit(const Key* key, const Scalar<std::string>& value) override;
  virtual void visit(const Key* key, const Object& value) override;

protected:
  void addParam(const std::string& name, const AbstractStruct::Field* field,
                const std::string& type_name);

  template <typename T, typename P>
  void addConstraints(const std::string& name,
                      const AbstractStruct::Field* field,
                      std::vector<P> dynamic_reconfigure::Config::* p);

  dynamic_reconfigure::ConfigDescription* description_;
  dynamic_reconfigure::Group* top_group_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__ROS_DYNAMIC_CONFIG_DESCRIBER_H_
