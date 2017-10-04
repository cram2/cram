#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright (c) 2017, Christopher Pollok <cpollok@uni-bremen.de>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Institute for Artificial Intelligence/
#       Universitaet Bremen nor the names of its contributors may be used to
#       endorse or promote products derived from this software without
#       specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import re
import sys
import xml.etree.ElementTree as et

FORMAT_1_DEPEND_TAGS = set([
    # "buildtool_depend",
    "build_depend",
    "run_depend",
    "test_depend"
])

FORMAT_2_DEPEND_TAGS = set([
    "depend",
    # "buildtool_depend",
    "build_depend",
    "exec_depend",
    "test_depend",
    "doc_depend"
])

# Packages in the source directory being checked, mapped to their path (relative from the cwd when executing)
pkg_paths = {}

# Packages mapped to a tuple of their dependencies as lists (package.xml, asdf)
pkg_deps = {}

# Packages in which 
errored = []


def get_package_name(path):
    xml_path = os.path.join(path, 'package.xml')
    e = et.parse(xml_path).getroot()
    return e.find('name').text

def get_asdf_name(pkg_name):
    return pkg_name.replace("_", "-")

def get_asdf_path(pkg_name, pkg_path):
    for dirpath, dirnames, filenames in os.walk(pkg_path):
        asdf_name = get_asdf_name(pkg_name) + ".asd"
        if asdf_name in filenames:
            return os.path.join(dirpath, asdf_name)

def get_xml_dependencies(xml_path):
    # Choose the right set of dependency tags
    xml = et.parse(xml_path).getroot()
    depend_tags = set()
    if "format" in xml.attrib and xml.attrib["format"] == "2":
        depend_tags.update(FORMAT_2_DEPEND_TAGS)
    else:
        depend_tags.update(FORMAT_1_DEPEND_TAGS)

    # Find all elements containing dependencies
    dependency_elements = reduce(lambda x, y: x + xml.findall(y), depend_tags, [])

    # Map the elements to their content text, which is the name of the dependency
    dependencies = map(lambda x: x.text, dependency_elements)
    return dependencies


def crawl():
    """Traverse the repository and collect all packages in pkg_paths."""
    global pkg_paths
    for dirpath, dirnames, filenames in os.walk('.'):
        if '.git' in dirnames:
            # don't look into .git directories
            del dirnames[dirnames.index('.git')]

        if 'CMakeLists.txt' in filenames and 'package.xml' in filenames:
            # found a package
            pkg_paths[get_package_name(dirpath)] = dirpath
            dirnames[:] = []

def parse_simple_asdf(asdf):
    depends_on = re.search(r"\((?:asdf:)*defsystem.*?:depends-on\s*\((.*?)\).*\)", asdf, re.S)
    if depends_on and depends_on.groups():
        return depends_on.groups()[0]


def get_asdf_dependencies(filepath):
    """Parse the dependencies in an asdf file."""
    with open(filepath, 'r') as file:
        content = file.read()

    deps_content = parse_simple_asdf(content)
    if deps_content == None:
        return []
    while deps_content.find(';') != -1:
        start = deps_content.find(';')
        end = deps_content.find("\n", start)
        deps_content = deps_content[:start] + deps_content[end:]

    dependencies = deps_content.split()
    return dependencies

def normalize_asdf_deps(deps):
    def normalize_dep(dep):
        # dependencies = map(lambda x: x[1:] if x.startswith(':') else x, dependencies)
        if dep.startswith('#:'):
            dep = dep[2:]

        if dep.startswith(':'):
            dep = dep[1:]

        if dep.endswith("-msg") or dep.endswith("-srv"):
            dep = dep[:-4]
        dep = dep.replace("-", "_")
        return dep
    deps = [x for x in deps if not x.startswith('#+')]
    return map(normalize_dep, deps)

def check_pkg_xmls():
    """Iterate over pkg_paths and check if the system and package.xml correpond to each other."""
    global errored
    for pkg, path in pkg_paths.items():
        try:
            xml_path = os.path.join(path,"package.xml")
            xml_deps = set(get_xml_dependencies(xml_path))
            asdf_path = get_asdf_path(pkg, path)  # os.path.join(path, get_asdf_name(pkg) + ".asd")
            
            if not asdf_path:
                errored.append(pkg)
                continue
            asdf_deps = set(normalize_asdf_deps(get_asdf_dependencies(asdf_path)))
            if pkg in asdf_deps:
                asdf_deps.remove(pkg)

            pkg_deps[pkg] = (xml_deps, asdf_deps)
            
        except IOError:
            errored.append(pkg)
            continue

def print_results():
    all_good = []

    result = ""
    for pkg, (xml_deps, asdf_deps) in pkg_deps.items():
        missing_in_xml = asdf_deps.difference(xml_deps)
        missing_in_asdf = xml_deps.difference(asdf_deps)
        if missing_in_xml or missing_in_asdf:
            result += "\n\n{}\n".format(pkg)
            result += "-"*len(pkg)
            result += "\npackage.xml Dependencies:\n"
            result += str(sorted(xml_deps))
            result += "\n\nasdf-System Dependencies:\n"
            result += str(sorted(asdf_deps))
            if missing_in_xml:
                result += "\n\nMissing in package.xml:\n"
                result += str(sorted(missing_in_xml))
            if missing_in_asdf:
                result += "\n\nMissing in asdf:\n"
                result += str(sorted(missing_in_asdf))
            result += "\n\n{}".format("="*40)
        else:
            all_good.append(pkg)
    result += "\nEverything's fine for these packages:\n"
    result += str(sorted(all_good))

    with open("latest_asdf_check", 'w') as file:
        file.write(result)

    print "\nResults:"
    print result

    print "Could not open some file in:"
    print errored


if __name__ == '__main__':
    crawl()
    check_pkg_xmls()
    print_results()
