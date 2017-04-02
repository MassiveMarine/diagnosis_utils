import re
from tug_cfg.model import Cfg, Param
from xml.etree import ElementTree


class ErrorHandler(object):
    def handle_parse_error(self, e):
        raise e

    def handle_unknown_tag(self, elem):
        print('Warning: Unknown tag "%s"; ignored' % elem.tag)

    def handle_unknown_attribute(self, elem, name):
        print('Warning: unknown attribute "%s" in "%s" element; ignored' % (name, elem.tag))

    def handle_invalid_attribute_value(self, elem, name, value):
        print('Warning: attribute "%s" in "%s" element has invalid value; ignored' % (name, elem.tag))

    def handle_unexpected_text(self, elem):
        print('Warning: "%s" element contains text; ignored' % elem.tag)


class ElementSpec(object):
    def __init__(self, class_, text_only=False, attributes={}, children={}):
        self.class_ = class_
        self.text_only = text_only
        self.attributes = attributes
        self.children = children


class Loader(object):
    _SPECS = dict(
        cfg=ElementSpec(Cfg, attributes=dict(
            extends=None,
        ), children=dict(
            param=ElementSpec(Param, attributes=dict(
                name=re.compile('^[a-z][a-z0-9]*(_[a-z0-9]+)*$'),
                type=re.compile('^(bool|double|int|str)$'),  # Only simple types for now
                unit=None,
                default=None,
                description=None,
                min=None,
                max=None,
                dynamic=re.compile('^(true|false)$'),
                level=re.compile('^(0|[1-9][0-9]*)$'),
                choices=None,
                suggestions=None,
                ignored=re.compile('^(true|false)$'),
            ))
        ))
    )

    def __init__(self, error_handler=ErrorHandler()):
        self._error_handler = error_handler

    def load(self, file_name):
        try:
            root_elem = ElementTree.parse(file_name).getroot()
        except Exception as e:
            return self._error_handler.handle_parse_error(e)
        else:
            return self._process_elem(None, root_elem, self._SPECS)

    def _process_elem(self, parent_object, elem, specs):
        if elem.tag in specs:
            spec = specs[elem.tag]
            object_ = spec.class_()
            # Go through attributes:
            for k, v in elem.items():
                self._process_attribute(object_, elem, spec.attributes, k, v)
            if elem.text is not None and not elem.text.isspace():
                if len(elem) == 0 and hasattr(object_, 'text'):
                    object_.text = elem.text
                else:
                    self._error_handler.handle_unexpected_text(elem)
            for child_elem in elem:
                child_object = self._process_elem(object_, child_elem)
                if child_elem.text is not None and not child_elem.text.isspace() and len(child_elem) == 0:
                    self._process_attribute(child_elem.tag, child_elem.text)
                else:
                    self._process_attribute(child_elem.tag, child_elem)
                if child_elem.tail is not None and not child_elem.tail.isspace():
                    # Tail text is actually part of parent element, so show it as warning of parent element:
                    print('Warning: "%s" element contains text; ignored' % elem.tag)
        self._error_handler.handle_unknown_tag(elem)

    def _process_attribute(self, object_, elem, specs, k, v):
        if k in specs:
            spec = specs[k]
            if spec is None or spec.match(v):
                setattr(object_, k, v)
            else:
                self._error_handler.handle_invalid_attribute_value(elem, k, v)
            return
        self._error_handler.handle_unknown_attribute(elem, k)
