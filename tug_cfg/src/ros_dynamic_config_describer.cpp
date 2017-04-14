#include <tug_cfg/ros_dynamic_config_describer.h>
#include <sstream>
#include <tug_cfg/key.h>
#include <tug_cfg/scalar.h>

namespace tug_cfg
{
RosDynamicConfigDescriber::RosDynamicConfigDescriber(dynamic_reconfigure::ConfigDescription* description)
  : description_(description)
{
  dynamic_reconfigure::Group group;
  group.name = "Default";
  group.type = "";
  group.parent = 0;
  group.id = 0;
  description_->groups.push_back(group);
  top_group_ = &description_->groups.back();
}

RosDynamicConfigDescriber::~RosDynamicConfigDescriber()
{
}

void RosDynamicConfigDescriber::visit(const Key* key, const AbstractMap& value)
{
  // TODO
}

void RosDynamicConfigDescriber::visit(const Key* key, const AbstractSequence& value)
{
  // TODO
}

void RosDynamicConfigDescriber::visit(const Key* key, const AbstractStruct& value)
{
  if (key == nullptr)
  {
    top_group_ = &description_->groups.at(0);
  }
  else
  {
    dynamic_reconfigure::Group group;
    std::ostringstream name;
    name << key;
    group.name = name.str();
    group.type = "collapse";
    group.parent = top_group_->id;
    group.id = description_->groups.size();
    description_->groups.push_back(group);
    top_group_ = &description_->groups.back();
  }

  value.acceptElements(key, *this);

  top_group_ = &description_->groups.at(top_group_->parent);
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<bool>& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream s;
    s << key;
    std::string name(s.str());
    addParam(name, field, "bool");
    addConstraints<bool>(name, field, &dynamic_reconfigure::Config::bools);
  }
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<double>& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream s;
    s << key;
    std::string name(s.str());
    addParam(name, field, "double");
    addConstraints<double>(name, field, &dynamic_reconfigure::Config::doubles);
  }
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<int>& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream s;
    s << key;
    std::string name(s.str());
    addParam(name, field, "int");
    addConstraints<int>(name, field, &dynamic_reconfigure::Config::ints);
  }
}

void RosDynamicConfigDescriber::visit(const Key* key, const Scalar<std::string>& value)
{
  const AbstractStruct::Field* field = key->asPtr<const AbstractStruct::Field>();
  if (field != nullptr && field->dynamic)
  {
    std::ostringstream s;
    s << key;
    std::string name(s.str());
    addParam(name, field, "str");
    addConstraints<std::string>(name, field, &dynamic_reconfigure::Config::strs);
  }
}

void RosDynamicConfigDescriber::visit(const Key* key, const Object& value)
{
  // TODO: warn
}

void RosDynamicConfigDescriber::addParam(const std::string& name,
                                         const AbstractStruct::Field* field,
                                         const std::string& type_name)
{
  dynamic_reconfigure::ParamDescription param_desc;
  param_desc.name = name;
  param_desc.type = type_name;
  param_desc.level = field->level;
  param_desc.description = field->description;
  if (!field->unit.empty())
  {
    param_desc.description += " (in " + field->unit + ")";
  }
  top_group_->parameters.push_back(param_desc);
}

template <typename T, typename P>
void RosDynamicConfigDescriber::addConstraints(
    const std::string& name, const AbstractStruct::Field* field,
    std::vector<P> dynamic_reconfigure::Config::* p)
{
  const AbstractStruct::ScalarField<T>& scalar_field(
        dynamic_cast<const AbstractStruct::ScalarField<T>&>(*field));
  P param;
  param.name = name;
  param.value = scalar_field.default_value;
  (description_->dflt.*p).push_back(param);
  param.value = scalar_field.min;
  (description_->min.*p).push_back(param);
  param.value = scalar_field.max;
  (description_->max.*p).push_back(param);
  // TODO: add choices (as enum)
}

}  // namespace tug_cfg
