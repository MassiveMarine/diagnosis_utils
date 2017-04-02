import re
from tug_cfg import model
from xml.etree import ElementTree


class ErrorHandler(object):
    def handle_parse_error(self, e):
        raise e

    def handle_unknown_tag(self, elem):
        print('Warning: Unknown tag "%s"; ignored' % elem.tag)

    def handle_unknown_attribute(self, elem, name):
        print('Warning: unknown attribute "%s" in "%s" element; ignored' % (name, elem.tag))

    def handle_unexpected_text(self, elem):
        print('Warning: "%s" element contains text; ignored' % elem.tag)


class Loader(object):
    _NAME_RE = re.compile('^[a-z]+(-[a-z]+)*$')

    def __init__(self, error_handler=ErrorHandler()):
        self._error_handler = error_handler

    def load(self, file_name):
        try:
            root_elem = ElementTree.parse(file_name).getroot()
        except Exception as e:
            return self._error_handler.handle_parse_error(e)
        else:
            return self._process_elem(root_elem)

    def _process_elem(self, parent_object, elem):
        if re.match(self._NAME_RE, elem.tag):
            class_name = ''.join(s.capitalize() for s in elem.tag.split('-'))
            if hasattr(model, class_name):
                class_ = getattr(model, class_name)
                if isinstance(class_, type):
                    object_ = class_(self._error_handler)
                    # Go through attributes:
                    for k, v in elem.items():
                        self._process_attribute(object_, elem, k, v)
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

    def _process_attribute(self, object_, elem, k, v):
        if re.match(self._NAME_RE, k):
            attr_name = '_'.join(k.split('-'))
            if hasattr(object_, attr_name):
                setattr(object_, attr_name, v)
                return
        self._error_handler.handle_unknown_attribute(elem, k)
