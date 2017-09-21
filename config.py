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
import sys
import yaml
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


# Packages in the source directory being configured, mapped to their path (relative from the cwd when executing)
pkg_paths = {}

# Packages to be whitelisted by config()
wl_packages = []


##################
# Help functions #
##################

def blacklist(dirpath):
    dir_content = os.listdir(dirpath)
    if 'CATKIN_IGNORE' not in dir_content:
        # open(os.path.join(dirpath, 'CATKIN_IGNORE'), 'w')
        pass
    print "Blacklisted: " + dirpath

def whitelist(dirpath):
    dir_content = os.listdir(dirpath)
    if 'CATKIN_IGNORE' in dir_content:
        # os.remove(os.path.join(dirpath, 'CATKIN_IGNORE'))
        pass
    print "Whitelisted: " + dirpath


def get_package_name(path):
    xml_path = os.path.join(path, 'package.xml')
    e = et.parse(xml_path).getroot()
    return e.find('name').text

def get_dependencies(xml):
    # Choose the right set of dependency tags
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


##################
# Main functions #
##################

# TODO: Actually check something.
def check_cwd():
    """Check if the script is run in the cram repository."""
    return True

def crawl():
    """Traverse the repository and collect all pacakges in pkg_paths."""
    global pkg_paths
    for dirpath, dirnames, filenames in os.walk('.'):
        if '.git' in dirnames:
            # don't look into .git directories
            del dirnames[dirnames.index('.git')]

        if 'CMakeLists.txt' in filenames and 'package.xml' in filenames:
            # found a package
            pkg_paths[get_package_name(dirpath)] = dirpath
            dirnames[:] = []

def check_package_names():
    """Check if any package name is different from the name of the directory containing it."""
    for pkg_name, dirpath in pkg_paths.items():
        dirname = os.path.basename(dirpath)
        if pkg_name != dirname:
            print "WARNING: Package name and name of containing directory differ: Package '{}' in directory '{}'!".format(pkg_name, dirname)

def check_for_high_level_packages(pkgs):
    """Check if the given packages are existent in the repository."""
    high_lvl_pkgs_in_dir = [x in pkg_paths.keys() for x in pkgs]
    if not all(high_lvl_pkgs_in_dir):
        print("WARNING: Not all packages you passed are existent in the current directory.")
        for pkg_not_found in [x for x in pkgs if not high_lvl_pkgs_in_dir[pkgs.index(x)]]:
            print("WARNING: {} does not designate a package inside {}.".format(pkg_not_found, os.getcwd()))

def build_profile(pkgs):
    """Loop up the dependecies of the packages in pkgs until reaching the lowest level and thus finding no more dependencies."""
    global wl_packages
    wl_packages = set()
    wl_packages.update(pkgs)
    while len(pkgs) != 0:
        for pkg in pkgs:
            xml_path = os.path.join(pkg_paths[pkg], 'package.xml')
            pkg_xml = et.parse(xml_path).getroot()
            deps = get_dependencies(pkg_xml)
            wl_packages.update(deps)
        pkgs = set(deps).intersection(pkg_paths.keys())

def config():
    """Traverse the repository and black-/whitelist the packages/directories."""
    for dirpath, dirnames, filenames in os.walk('.'):
        pkgs_in_dir = [x for x in pkg_paths.keys() if pkg_paths[x].startswith(dirpath)]
        if len(pkgs_in_dir) != 0:
            if not any([x in wl_packages for x in pkgs_in_dir]):
                blacklist(dirpath)
                ## Uncomment this, if you want to remove CATKIN_IGNOREs in the packages when blacklisting the whole dir. Comment the next line then though.
                # wl_packages.update(pkgs_in_dir)
                dirnames[:] = []
            else:
                whitelist(dirpath)

        elif 'CMakeLists.txt' in filenames and 'package.xml' in filenames:
            # found a package, don't recurse deeper
            dirnames[:] = []
            basepath, dirname = os.path.split(dirpath)
            if any(map(lambda x: x == dirname, wl_packages)):
                whitelist(dirpath)
            else:
                blacklist(dirpath)

        if '.git' in dirnames:
            # don't look into .git directories
            del dirnames[dirnames.index('.git')]

def usage():
    usage_txt = """Usage:
    python config.py <package> [*<package>]

    The script has to be run inside the top-level directory of the CRAM repository.
    You can only pass packages which are inside the CRAM repository, otherwise the script can't know for which dependencies to configure.
    """
    return usage_txt


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] in ["h", "-h", "help", "--help"] or not check_cwd():
        print(usage())
        sys.exit()

    crawl()
    check_package_names()

    if not check_for_high_level_packages(sys.argv[1:]):
        print(usage())
        sys.exit()

    build_profile(sys.argv[1:])
    config()
