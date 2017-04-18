#include <tug_cfg/ros_dynamic_config_writer.h>
#include <ros/console.h>
#include <tug_cfg/collection.h>
#include <tug_cfg/struct.h>

namespace tug_cfg
{
RosDynamicConfigWriter::RosDynamicConfigWriter(dynamic_reconfigure::Config* config)
  : config_(config)
{
}

RosDynamicConfigWriter::~RosDynamicConfigWriter()
{
}

void RosDynamicConfigWriter::visit(const Key* key, const AbstractMap& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ROS_WARN_NAMED("RosDynamicConfigWriter", "tried to write map");
  }
}

void RosDynamicConfigWriter::visit(const Key* key, const AbstractSequence& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    ROS_WARN_NAMED("RosDynamicConfigWriter", "tried to write sequence");
  }
}

void RosDynamicConfigWriter::visit(const Key* key, const AbstractStruct& value)
{
  value.acceptElements(key, *this);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<bool>& value)
{
  visitScalar(key, value, config_->bools);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<double>& value)
{
  visitScalar(key, value, config_->doubles);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<int>& value)
{
  visitScalar(key, value, config_->ints);
}

void RosDynamicConfigWriter::visit(const Key* key, const Scalar<std::string>& value)
{
  visitScalar(key, value, config_->strs);
}

void RosDynamicConfigWriter::visit(const Key* key, const Object& value)
{
  ROS_ERROR_NAMED("RosDynamicConfigWriter", "unsupported type");
}

template <typename T, typename P>
void RosDynamicConfigWriter::visitScalar(const Key* key, const Scalar<T>& value,
                                         std::vector<P>& values)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream name;
    name << key;
    P p;
    p.name = name.str();
    p.value = static_cast<const typename P::_value_type&>(
          static_cast<const T&>(value));
    values.push_back(p);
  }
}
}  // namespace tug_cfg
