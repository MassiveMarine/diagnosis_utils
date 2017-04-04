import os.path
import re
import yaml
from .model import Cfg, Param, CfgType, ListType, MapType, ScalarType


class ErrorHandler(object):
    def handle_parse_error(self, e):
        raise e

    def handle_invalid_cfg_name(self, cfg_name):
        print('Warning: "%s" is an invalid configuration name; ignored' % (cfg_name,))

    def handle_invalid_parameter_name(self, parameter_name):
        print('Warning: parameter "%s" has invalid name; ignored' % (parameter_name,))

    def handle_unknown_attribute(self, parameter_name, attribute_name):
        print('Warning: parameter "%s" has unknown attribute "%s"; ignored' % (parameter_name, attribute_name))

    def handle_invalid_parameter_spec(self, parameter_name):
        print('Warning: specification of parameter "%s" is invalid; ignored' % parameter_name)

    def handle_invalid_attribute_type(self, parameter_name, attribute_name, expected_type):
        print('Warning: attribute "%s" of parameter "%s" has invalid type; should be %s; ignored'
              % (attribute_name, parameter_name, expected_type))

    def handle_invalid_attribute_value(self, parameter_name, attribute_name, attribute_value):
        print('Warning: attribute "%s" of parameter "%s" has invalid value %r; ignored'
              % (attribute_name, parameter_name, attribute_value))

    def handle_unexpected_doc(self, doc):
        print('Warning: file contains superfluous document; ignored')


class AttributeSpec(object):
    def __init__(self, type_, pattern=None):
        self.type = type_
        self.pattern = re.compile(pattern) if pattern else None


class Loader(object):
    _CFG_NAME_PATTERN = re.compile('^[A-Z][A-Za-z0-9]*$')
    _NAME_PATTERN = re.compile('^[a-z][a-z0-9]*(_[a-z0-9]+)*$')
    _TYPE_PATTERN = re.compile('^((?P<scalar>%s)|(?P<cfg>[a-z][a-z0-9_]*/[A-Z][A-Za-z0-9]*))'
                               '(?P<collection>(\\[(|%s)\\])*)$' % ('|'.join(ScalarType.ALL_NAMES),
                                                                    '|'.join((ScalarType.INT, ScalarType.STR))))

    _ATTRIBUTE_SPECS = dict(
        type=AttributeSpec(str),
        unit=AttributeSpec(str),
        default=AttributeSpec(object),
        description=AttributeSpec(str),
        min=AttributeSpec(object),
        max=AttributeSpec(object),
        dynamic=AttributeSpec(bool),
        level=AttributeSpec(int),
        choices=AttributeSpec(list),
        suggestions=AttributeSpec(list),
        ignored=AttributeSpec(bool),
    )

    def __init__(self, error_handler=ErrorHandler()):
        self._error_handler = error_handler

    def load(self, package_name, file_name):
        result = None
        cfg_name = os.path.splitext(os.path.basename(file_name))[0]
        if self._CFG_NAME_PATTERN.match(cfg_name):
            try:
                with open(file_name) as f:
                    docs = list(yaml.safe_load_all(f))
            except Exception as e:
                self._error_handler.handle_parse_error(e)
            else:
                for i, doc in enumerate(docs):
                    if i == 0:
                        result = self._process_doc(package_name, cfg_name, doc)
                    else:
                        self._error_handler.handle_unexpected_doc(doc)
        else:
            self._error_handler.handle_invalid_cfg_name(cfg_name)
        return result

    def _process_doc(self, package_name, cfg_name, doc):
        cfg = Cfg(package_name, cfg_name)
        for name, attributes in doc.items():
            if self._NAME_PATTERN.match(name):
                param = Param(name)
                for k, v in attributes.items():
                    try:
                        spec = self._ATTRIBUTE_SPECS[k]
                    except KeyError:
                        self._error_handler.handle_unknown_attribute(name, k)
                    else:
                        if isinstance(v, spec.type) and (spec.pattern is None or spec.pattern.match(v)):
                            if k == 'type':
                                type_ = self._parse_type(v)
                                if type_:
                                    setattr(param, k, type_)
                                else:
                                    self._error_handler.handle_invalid_attribute_value(name, k, v)
                            else:
                                setattr(param, k, v)
                        else:
                            self._error_handler.handle_invalid_attribute_value(name, k, v)
                if (param.name and param.type and param.default is not None
                        and not (param.choices and param.suggestions)):
                    cfg.parameters.append(param)
                else:
                    self._error_handler.handle_invalid_parameter_spec(name)
            else:
                self._error_handler.handle_invalid_parameter_name(name)
        return cfg

    def _parse_type(self, s):
        type_ = None
        match = self._TYPE_PATTERN.match(s)
        if match:
            scalar_type_name = match.group('scalar')
            cfg_type_name = match.group('cfg')
            collection_decls = match.group('collection')
            if scalar_type_name:
                type_ = ScalarType(scalar_type_name)
            elif cfg_type_name:
                type_ = CfgType(*cfg_type_name.split('/'))
            if collection_decls:
                for collection_decl in collection_decls[1:-1].split(']['):
                    if collection_decl == '':
                        type_ = ListType(type_)
                    elif collection_decl in ScalarType.ALL_NAMES:
                        type_ = MapType(ScalarType(collection_decl), type_)
                    else:
                        return None
        return type_