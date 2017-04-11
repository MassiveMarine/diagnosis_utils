#include <tug_cfg/collection.h>
#include <tug_cfg/visitor.h>

namespace tug_cfg
{
void AbstractSequence::accept(Key* key, Visitor& visitor)
{
  visitor.visit(key, *this);
}

void AbstractSequence::accept(const Key* key, ConstVisitor& visitor) const
{
  visitor.visit(key, *this);
}

void AbstractMap::accept(Key* key, Visitor& visitor)
{
  visitor.visit(key, *this);
}

void AbstractMap::accept(const Key* key, ConstVisitor& visitor) const
{
  visitor.visit(key, *this);
}
}  // namespace tug_cfg
