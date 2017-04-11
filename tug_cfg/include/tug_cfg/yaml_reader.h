#ifndef _TUG_CFG__YAML_READER_H_
#define _TUG_CFG__YAML_READER_H_

#include <memory>
#include <queue>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
class YamlReader : public Visitor
{
public:
  YamlReader(const std::string& file_path);

  virtual void visit(Key* key, AbstractMap& value) override;
  virtual void visit(Key* key, Scalar<bool>& value) override;
  virtual void visit(Key* key, Scalar<double>& value) override;
  virtual void visit(Key* key, Scalar<int>& value) override;
  virtual void visit(Key* key, Scalar<std::string>& value) override;
  virtual void visit(Key* key, AbstractStruct& value) override;
  virtual void visit(Key* key, AbstractSequence& value) override;
  virtual void visit(Key* key, Object& value) override;

protected:
  class Context;
  friend class Context;

  std::queue<std::shared_ptr<Context>> documents_;
  Context* top_;
};
}  // namespace tug_cfg

#endif  // _TUG_CFG__YAML_READER_H_
