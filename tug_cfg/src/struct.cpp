#include <tug_cfg/struct.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
AbstractStruct::Field::Field(const std::string& name_, const std::string& unit_,
                             const std::string& description_, bool dynamic_,
                             int level_, bool ignored_)
  : name(name_), unit(unit_), description(description_), dynamic(dynamic_),
    level(level_), ignored(ignored_)
{
}

void AbstractStruct::accept(Key* key, Visitor& visitor)
{
  visitor.visit(key, *this);
}

void AbstractStruct::accept(const Key* key, ConstVisitor& visitor) const
{
  visitor.visit(key, *this);
}

template <>
void ScalarKey<const AbstractStruct::Field*>::format(std::ostream& s) const
{
  if (parent_ != nullptr)
  {
    s << '.';
  }
  s << key_->name;
}
}
