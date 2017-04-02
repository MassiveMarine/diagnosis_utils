

class CfgComponent(object):
    pass


class Cfg(CfgComponent):
    def __init__(self):
        self._extends = None
        self._params = []

    @property
    def extends(self):
        return self._extends

    @extends.setter
    def extends(self, value):
        if self._extends is not None:
            raise Warning('Trying to set "extends" property more than once; ignored')
        # TODO: parse and evaluate type
        self._extends = value

    @property
    def param(self):
        return self._params

    @param.setter
    def param(self, value):
        if isinstance(value, ElementTree.Element):
            p = Param()
            p.init_from_xml(value)
            self._params.append(p)
        else:
            raise Warning('Invalid contents for "param"; ignored')


class Param(CfgComponent):
    def __init__(self):
        self.name = None
        self.type = None
        self.unit = None
        self.default = None
        self.description = None
        self.min = None
        self.max = None
        self.dynamic = None
        self.level = None
        self.choices = None
        self.suggestions = None
        self.ignored = False

    def init_from_xml(self, elem):
        super(Param, self).init_from_xml(elem)
        if self.name is None:
            raise ValueError('"param" tag is lacking name')
        elif not self.ignored:
            if self.type is None or (self.type.isalpha() and self.default is None):
                raise ValueError('"param" tag named "%s" is lacking required attributes' % self.name)
