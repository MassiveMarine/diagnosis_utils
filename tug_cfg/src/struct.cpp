#include <tug_cfg/struct.h>

namespace tug_cfg
{

AbstractStruct::FieldInfo::FieldInfo(const std::string& unit_,
                             const std::string& description_, bool dynamic_,
                             int level_, bool ignored_)
  : unit(unit_), description(description_), dynamic(dynamic_),
    level(level_), ignored(ignored_)
{
}

void AbstractStruct::accept(Key& key, Visitor& visitor)
{
  visitor.visit(key, *this);
}

void AbstractStruct::accept(const Key& key, ConstVisitor& visitor) const
{
  visitor.visit(key, *this);
}

}
