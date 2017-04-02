#!/usr/bin/env python
from tug_cfg.loader import Loader


def main():
    try:
        l = Loader()
        l.load('Bar.cfg')
    except ValueError as e:
        print('Error: %s' % e.message)
    except Exception as e:
        print('Error: %s' % e)

if __name__ == '__main__':
    main()
