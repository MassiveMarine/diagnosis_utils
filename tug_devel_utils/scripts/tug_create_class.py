#!/usr/bin/env python

from __future__ import print_function

import os.path
import re
import rospkg
import string
import sys


QUALIFIED_CLASS_NAME_RE = re.compile('^(([a-z][a-z0-9_]*)::)*([A-Z][A-Za-z0-9]*)$')
CLASS_NAME_SEP_RE = re.compile('[A-Z][0-9]*[a-z]')


TEMPLATE_H = string.Template('''\
#ifndef _${NAMESPACE}__${CLASS_NAME}_H_
#define _${NAMESPACE}__${CLASS_NAME}_H_

namespace $namespace
{
class $ClassName
{
public:
  $ClassName();
  virtual ~$ClassName();

protected:
};
}  // namespace $namespace

#endif  // _${NAMESPACE}__${CLASS_NAME}_H_
''')


TEMPLATE_CPP = string.Template('''\
#include <$include_file_path>

namespace $namespace
{
$ClassName::$ClassName()
{
}

$ClassName::~$ClassName()
{
}
}  // namespace $namespace
''')


class Error(Exception):
    pass


def get_pkg_path(pkg_name):
    try:
        pkg_path = rospkg.RosPack().get_path(pkg_name)
    except rospkg.ResourceNotFound:
        raise Error('package "%s" not found' % pkg_name)
    if not os.path.isdir(pkg_path):
        raise Error('directory of package "%s" does not exist' % pkg_name)
    return pkg_path


def split_qualified_class_name(qualified_class_name):
    match = QUALIFIED_CLASS_NAME_RE.match(qualified_class_name)
    if not match:
        raise Error('"%s" is not a valid class name' % qualified_class_name)
    namespace = (match.group(1) or '').split('::')[:-1]
    class_name = match.group(3)
    class_name_parts = []
    start = 0
    for sep_match in CLASS_NAME_SEP_RE.finditer(class_name, 1):
        class_name_parts.append(class_name[start:sep_match.start()])
        start = sep_match.start()
    class_name_parts.append(class_name[start:])
    return namespace, class_name, '_'.join(p.lower() for p in class_name_parts)


def create_file(file_name, contents):
    if os.path.exists(file_name):
        raise Error('file "%s" already exists' % file_name)
    directory = os.path.dirname(file_name)
    if not os.path.isdir(directory):
        print('Creating directory "%s"' % directory)
        os.makedirs(directory)
    print('Creating file "%s"' % file_name)
    try:
        with open(file_name, 'w') as f:
            f.write(contents)
    except IOError as e:
        raise Error('could not write file: %s' % e)


def create_class(pkg_name, qualified_class_name):
    pkg_path = get_pkg_path(pkg_name)
    namespace, class_name, file_name = split_qualified_class_name(qualified_class_name)
    full_namespace = [pkg_name] + namespace

    include_file_path = [pkg_path, 'include'] + full_namespace + [file_name + '.h']
    source_file_path = [pkg_path, 'src'] + namespace + [file_name + '.cpp']

    substitutions = dict(
        include_file_path=os.path.join(*include_file_path[2:]),
        namespace='::'.join(full_namespace),
        NAMESPACE='__'.join(p.upper() for p in full_namespace),
        ClassName=class_name,
        CLASS_NAME=file_name.upper(),
    )

    create_file(os.path.join(*include_file_path), TEMPLATE_H.substitute(substitutions))
    create_file(os.path.join(*source_file_path), TEMPLATE_CPP.substitute(substitutions))

    print('Add the following line to your CMakeLists.txt:')
    print('  %s include/${PROJECT_NAME}/%s' % ('/'.join(source_file_path[1:]), '/'.join(include_file_path[3:])))


def main():
    try:
        if len(sys.argv) < 3:
            raise Error('too few arguments given')
        create_class(sys.argv[1], sys.argv[2])
    except Error as e:
        print('Error: %s' % e, file=sys.stderr)
        print('Usage: %s <pkg_name> <ClassName>' % os.path.basename(sys.argv[0]), file=sys.stderr)
        print('or:    %s <pkg_name> <additional_namespace>::<ClassName>' % os.path.basename(sys.argv[0]), file=sys.stderr)


if __name__ == '__main__':
    main()
