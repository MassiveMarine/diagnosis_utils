

class CfgComponent(object):
    pass


class Cfg(CfgComponent):
    def __init__(self):
        self.extends = None


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


class Description(CfgComponent):
    def __init__(self):
        self.text = None
