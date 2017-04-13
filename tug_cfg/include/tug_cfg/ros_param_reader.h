#ifndef _TUG_CFG__ROS_PARAM_READER_H_
#define _TUG_CFG__ROS_PARAM_READER_H_

#include <ros/node_handle.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class RosParamReader : public Visitor
{
public:
  RosParamReader(const ros::NodeHandle& node_handle, const std::string& key=std::string());
  RosParamReader(const XmlRpc::XmlRpcValue& value);
  virtual ~RosParamReader();

  virtual void visit(Key* key, AbstractMap& value) override;
  virtual void visit(Key* key, AbstractSequence& value) override;
  virtual void visit(Key* key, AbstractStruct& value) override;
  virtual void visit(Key* key, Scalar<bool>& value) override;
  virtual void visit(Key* key, Scalar<double>& value) override;
  virtual void visit(Key* key, Scalar<int>& value) override;
  virtual void visit(Key* key, Scalar<std::string>& value) override;
  virtual void visit(Key* key, Object& value) override;

protected:
  class Context
  {
  public:
    Context(RosParamReader* reader_);
    ~Context();

    bool enter(Key* key);

    RosParamReader* reader;
    Context* parent;
    XmlRpc::XmlRpcValue* node;
    XmlRpc::XmlRpcValue::iterator it;
  };

  friend class Context;

  Context* top_;
  XmlRpc::XmlRpcValue root_node_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__ROS_PARAM_READER_H_
