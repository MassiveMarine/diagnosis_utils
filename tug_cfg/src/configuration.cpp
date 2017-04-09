#include <tug_cfg/configuration.h>
#include <tug_cfg/default_constrainer.h>
#include <tug_cfg/key.h>

// We should definitely use accept(void, visitor) here because else, visitor
// doesn't know that we are accepting e.g. a struct, and cannot check for
// superfluous parameters:
namespace tug_cfg
{
void constrain(Object& value)
{
  constrain(value, DefaultConstrainer());
}

void constrain(Object& value, Visitor& constrainer)
{
  value.accept(ScalarKey<void>(), constrainer);
}

void load(Object& value, Visitor& source)
{
  load(value, source, DefaultConstrainer());
}

void load(Object& value, Visitor& source, Visitor& constrainer)
{
  value.accept(ScalarKey<void>(), source);
  value.accept(ScalarKey<void>(), constrainer);
}

void store(const Object& value, ConstVisitor& sink)
{
  object.accept(ScalarKey<void>(), sink);
}
}
