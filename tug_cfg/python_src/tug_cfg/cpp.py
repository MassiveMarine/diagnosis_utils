import math
import string
import textwrap
from .model import ScalarType, MapType, ListType, CfgType

_HEADER_TEMPLATE = string.Template('''\
#ifndef _${NAMESPACE}__${CLASS_NAME}_H_
#define _${NAMESPACE}__${CLASS_NAME}_H_

#include <limits>
#include <$gen_namespace/scalar.h>
#include <$gen_namespace/struct.h>

namespace $namespace
{
class $ClassName
{
public:
$fields

  //============================================================================
  // Magic starts here
  //============================================================================

  typedef $gen_namespace::Struct<$ClassName> Struct;

  $ClassName()
    : $initialization
  {
  }
  
  operator Struct::Type()
  {
    return Struct::Type(*this);
  }

  static const Struct::Type& getDefinition()
  {
    using namespace $gen_namespace;

    $field_specs

    static Struct::Type type("$namespace/$Name", {
      $field_names
    });

    return type;
  }
};
}  // namespace $namespace

#endif  // _${NAMESPACE}__${CLASS_NAME}_H_
''')

_FIELD_TEMPLATE = string.Template('''\
  /**
   * $doc
   */
  $type $name;
''')

_FIELD_SPEC_TEMPLATE = string.Template('''\
static Struct::FieldImpl<$meta_type> $field_name(
      "$name",
      Struct::FieldInfo($unit, $description, $dynamic, $level, $ignored),
      &$ClassName::$name);\
''')

_SCALAR_FIELD_SPEC_TEMPLATE = string.Template('''\
static Struct::FieldImpl<$meta_type> $field_name(
      "$name",
      Struct::ScalarFieldInfo<$type>($unit, $description, $dynamic, $level, $ignored,
        $default, $min, $max, {$choices}, {$suggestions}),
      &$ClassName::$name);\
''')


class CppParam(object):
    def __init__(self, class_name, param):
        self.name = param.name
        self.type = self._generate_type_name(param.type)
        self.default = self._format_value(param, param.default)
        self.min = self._format_value(param, param.min)
        self.max = self._format_value(param, param.max)
        self.choices = tuple(self._format_value(param, choice) for choice in (param.choices or ()))
        self.suggestions = tuple(self._format_value(param, choice) for choice in (param.suggestions or ()))
        self.description = param.description
        self.spec = self._generate_spec(class_name, param)

    def _generate_spec(self, class_name, param):
        if isinstance(param.type, ScalarType):
            return _SCALAR_FIELD_SPEC_TEMPLATE.substitute(
                type=self.type,
                meta_type=self._generate_meta_type(param.type),
                field_name='%s_field' % self.name,
                name=self.name,
                unit=self._format_string(param.unit),
                description=self._format_string(param.description),
                dynamic='true' if param.dynamic else 'false',
                level=param.level or 0,
                ignored='true' if param.ignored else 'false',
                default=self._format_value(param, param.default),
                min=self._format_min_max(param, 'min'),
                max=self._format_min_max(param, 'max'),
                choices=', '.join(self.choices),
                suggestions=', '.join(self.suggestions),
                ClassName=class_name,
            )
        else:
            return _FIELD_SPEC_TEMPLATE.substitute(
                meta_type=self._generate_meta_type(param.type),
                field_name='%s_field' % self.name,
                name=self.name,
                unit=self._format_string(param.unit),
                description=self._format_string(param.description),
                dynamic='true' if param.dynamic else 'false',
                level=param.level or 0,
                ignored='true' if param.ignored else 'false',
                ClassName=class_name,
            )

    def _generate_type_name(self, type_):
        if isinstance(type_, ScalarType):
            # if type_.name == ScalarType.ANY:
            #     return 'boost::any'
            if type_.name == ScalarType.STR:
                return 'std::string'
            return type_.name
        if isinstance(type_, MapType):
            return 'std::map<%s, %s> ' % (self._generate_type_name(type_.key_type),
                                          self._generate_type_name(type_.value_type))
        if isinstance(type_, ListType):
            return 'std::vector<%s> ' % (self._generate_type_name(type_.item_type),)
        if isinstance(type_, CfgType):
            return '%s::%sConfig' % (type_.package_name, type_.cfg_name)
        raise TypeError('Configuration model contains unknown type %r' % type_)

    def _generate_meta_type(self, type_):
        if isinstance(type_, ScalarType):
            return 'Scalar<%s>' % (self._generate_type_name(type_),)
        if isinstance(type_, MapType):
            return 'Map<%s, %s>' % (self._generate_type_name(type_.key_type), self._generate_meta_type(type_.value_type))
        if isinstance(type_, ListType):
            return 'Sequence<%s>' % (self._generate_meta_type(type_.item_type),)
        if isinstance(type_, CfgType):
            return 'Struct<%s>' % (self._generate_type_name(type_),)
        raise TypeError('Configuration model contains unknown type %r' % type_)

    def _format_value(self, param, value):
        if value is None:
            return None
        if isinstance(param.type, ScalarType):
            if param.type.name == ScalarType.BOOL:
                return self._format_bool(value)
            if param.type.name == ScalarType.STR:
                return self._format_string(value)
            return repr(value)
        #raise TypeError('Type %r cannot have values' % param.type)
        return None

    def _format_min_max(self, param, key):
        if isinstance(param.type, ScalarType):
            if param.type.name == ScalarType.BOOL:
                return self._format_bool(key == 'max') # Booleans can't have min or max values
            elif param.type.name in (ScalarType.DOUBLE, ScalarType.INT):
                value = getattr(param, key)
                if value is None:
                    return 'std::numeric_limits<%s>::%s()' % (self.type, key)
                elif math.isinf(value):
                    return '%sstd::numeric_limits<%s>::infinity()' % (('-' if value < 0 else ''), self.type)
                else:
                    return str(value)
            elif param.type.name == ScalarType.STR:
                return '""'  # Strings can't have min or max values

    def _format_bool(self, value):
        return 'true' if value else 'false'

    def _format_string(self, value):
        return '"%s"' % str(value or '').encode('string_escape')


class Generator(object):
    def generate(self, stream, cfg):
        class_name = cfg.name + 'Config'
        params = list(CppParam(class_name, p) for p in cfg.parameters)
        fields = []
        initialization = ',\n      '.join('%s(%s)' % (p.name, p.default) for p in params if p.default is not None)
        for p in params:
            doc = []
            if p.default is not None:
                doc.append('Default: %s' % p.default)
            if p.min is not None:
                doc.append('Minimum: %s' % p.min)
            if p.max is not None:
                doc.append('Maximum: %s' % p.max)
            if p.choices:
                doc.append('Choices: %s' % ', '.join(p.choices))
            if p.suggestions:
                doc.append('Suggestions: %s' % ', '.join(p.suggestions))
            if p.description:
                if doc:
                    doc.insert(0, '')  # Add empty line between description and rest of documentation
                doc.insert(0, p.description)
            fields.append(_FIELD_TEMPLATE.substitute(
                doc='\n   * '.join(textwrap.fill(d, subsequent_indent='   * ') for d in doc),
                type=p.type.strip(),
                name=p.name,
            ))
        stream.write(_HEADER_TEMPLATE.substitute(
            gen_namespace=self.__class__.__module__.split('.')[0],
            NAMESPACE=cfg.package_name.upper(),
            namespace=cfg.package_name,
            Name=cfg.name,
            CLASS_NAME=class_name.upper(),
            ClassName=class_name,
            fields='\n'.join(fields),
            initialization=initialization,
            field_specs='\n\n    '.join(p.spec for p in params),
            field_names=textwrap.fill(', '.join('&%s_field' % p.name for p in params),
                                      width=80, subsequent_indent='      '),
        ))
