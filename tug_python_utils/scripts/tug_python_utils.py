#!/usr/bin/env python


class YamlHelper():
    @staticmethod
    def get_param(config, name, default=None):
        if name in config:
            return config[name]

        if default is None:
            print "ERROR"
            raise KeyError("'" + str(name) + "' not found in config!")

        return default

    @staticmethod
    def get_param_with_default(config, name, default):
        return YamlHelper.get_param(config=config, name=name, default=default)
        # if name in config:
        #     return config[name]
        # return default

    @staticmethod
    def has_key(config, name):
        return name in config

    @staticmethod
    def has_param(config, name):
        return name in config
