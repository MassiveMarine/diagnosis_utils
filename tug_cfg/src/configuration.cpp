#include <tug_cfg/configuration.h>
#include <tug_cfg/default_constrainer.h>
#include <tug_cfg/key.h>
#include <tug_cfg/object.h>

// We should definitely use accept(void, visitor) here because else, visitor
// doesn't know that we are accepting e.g. a struct, and cannot check for
// superfluous parameters:
namespace tug_cfg
{
void constrain(Object& value)
{
  DefaultConstrainer constrainer;
  constrain(value, constrainer);
}

void constrain(Object& value, Visitor& constrainer)
{
  ScalarKey<void> key;
  value.accept(key, constrainer);
}

void load(Object& value, Visitor& source)
{
  DefaultConstrainer constrainer;
  load(value, source, constrainer);
}

void load(Object& value, Visitor& source, Visitor& constrainer)
{
  ScalarKey<void> key;
  value.accept(key, source);
  value.accept(key, constrainer);
}

void store(const Object& value, ConstVisitor& sink)
{
  ScalarKey<void> key;
  value.accept(key, sink);
}
}
