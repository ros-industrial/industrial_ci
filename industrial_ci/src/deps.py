#!/usr/bin/env python
from __future__ import print_function
from os import getenv, path
from catkin_pkg.packages import find_packages
from itertools import chain
from sys import stderr
from copy import copy

# read environment variables

workspace = getenv("CATKIN_WORKSPACE")
if workspace is None:
    print("please set $CATKIN_WORKSPACE", file=stderr)
    exit(1)
workspace_src=path.join(workspace, 'src')

strict_test_depends = getenv("STRICT_TEST_DEPENDS","") == "true"

target_pkgs = set(getenv("TARGET_PKGS","").split())

use_mockup = getenv("USE_MOCKUP","")
pkgs_whitelisted = set(getenv("BUILD_PKGS_WHITELIST","").split())
pkgs_blacklisted = set(getenv("BUILD_PKGS_BLACKLIST","").split())
downstream_pkgs = set(getenv("PKGS_DOWNSTREAM","").split())

tests_blacklisted = set(getenv("TEST_BLACKLIST","").split())

def get_target_repo_path(): # lazy read of TARGET_REPO_PATH
    target_repo_path = getenv("TARGET_REPO_PATH")
    if target_repo_path is None:
        print("please set $TARGET_REPO_PATH or $TARGET_PKGS", file=stderr)
        exit(1)
    return target_repo_path

if pkgs_whitelisted:
    pkgs_whitelisted |= downstream_pkgs # whitelist all explicit downstream packages

if len(target_pkgs) == 0:
    target_pkgs = set(p.name for p in find_packages(get_target_repo_path()).values())
    # match auto-detect target against whitelist and blacklist
    if pkgs_whitelisted:
        target_pkgs &= pkgs_whitelisted
    else:
        target_pkgs -= pkgs_blacklisted

if len(use_mockup) > 0:
    target_pkgs |= set(p.name for p in find_packages(path.join(get_target_repo_path(),use_mockup)).values())

if pkgs_whitelisted:
    pkgs_whitelisted |= target_pkgs # whitelist all target packages

pkgs_blacklisted -= pkgs_whitelisted # make whitelist and blacklist disjoint

known_pkgs = find_packages(workspace_src)
paths=dict((p.name, path) for (path,p) in known_pkgs.items())
workspace_pkgs = set(paths.keys())

recursive_deps = dict() # recursive dependencies in workspace

class Dependencies(object):
    def __init__(self, p=None):
        if p is None:
            self.build_deps = set()
            self.export_deps = set()
            self.test_deps = set()
            self.exec_deps = set()
            self.build_keys = set()
            self.export_keys = set()
            self.test_keys = set()
        else:
            self.build_deps = set(d.name for d in chain(p.build_depends, p.buildtool_depends)) & workspace_pkgs
            self.export_deps = set(d.name for d in chain(p.buildtool_export_depends, p.build_export_depends)) & workspace_pkgs
            if strict_test_depends:
                self.test_deps = set(d.name for d in chain(p.test_depends)) & workspace_pkgs
            else:
                self.test_deps = set(d.name for d in chain(p.test_depends, p.exec_depends)) & workspace_pkgs
            self.exec_deps = set(d.name for d in chain(p.exec_depends)) & workspace_pkgs
            self.build_keys = set(d.name for d in chain(p.build_depends, p.buildtool_depends)) # keys needed for build
            self.export_keys = set(d.name for d in chain(p.buildtool_export_depends, p.build_export_depends)) # keys needed for transitive build
            self.test_keys = set(d.name for d in chain(p.test_depends, p.exec_depends)) # keys needed for tests
    def join(self, other):
        self.build_deps |= other.build_deps | other.export_deps # export_deps turned into build_deps for dependent projects
        self.export_deps |= other.export_deps
        self.test_deps |= other.test_deps | other.exec_deps # exec_deps turned into test_deps for dependent projects
        self.exec_deps |= other.exec_deps
        self.build_keys |= other.build_keys | other.export_keys
        self.export_keys |= other.export_keys
        self.test_keys |= other.test_keys
        return self
    def getAllDeps(self):
        return self.build_deps | self.export_deps | self.test_deps | self.exec_deps

# dependencies of workspace packages within workspace
workspace_deps = dict((p.name, Dependencies(p)) for p in known_pkgs.values())

pkgs_blacklisted |= set(p.name for p in known_pkgs.values() if p.is_metapackage()) # ignore non-target metapackages

def get_recursive_deps(name):
    global recursive_deps, workspace_deps
    try:
        return recursive_deps[name]
    except KeyError:
        w = workspace_deps[name]
        deps = copy(w)
        for d in chain(w.getAllDeps()):
            deps.join(get_recursive_deps(d))
        recursive_deps[name] = deps
        return deps

upstream_pkgs = set()
downstream_tests = set()
target_tests = set(t for t in target_pkgs if len(workspace_deps[t].test_keys) > 0) - tests_blacklisted

for p in workspace_pkgs:
    deps = get_recursive_deps(p)
    if p in target_pkgs: # fill recursive dependencies of target packages
        upstream_pkgs |= deps.build_deps
        if p in target_tests:
            upstream_pkgs |= deps.test_deps
    if p in downstream_pkgs or len((deps.build_deps | deps.test_deps)  & target_pkgs) > 0: # is downstream package
        if p in pkgs_whitelisted or ( not pkgs_whitelisted and not p in pkgs_blacklisted): # match white and blacklist
            downstream_pkgs |= deps.build_deps
            downstream_pkgs.add(p)
            if len(deps.test_deps  & target_pkgs) > 0 and p not in tests_blacklisted:
                downstream_pkgs |= deps.test_deps
                downstream_tests.add(p)
    elif p in pkgs_whitelisted: # no recursive dependency, but whitelisted
        upstream_pkgs |= deps.build_deps
        upstream_pkgs.add(p)

all_pkgs = upstream_pkgs | downstream_pkgs | target_pkgs | target_tests | downstream_tests

skip_keys = set()
for a in all_pkgs:
    r = get_recursive_deps(a)
    skip_keys |= r.export_keys | r.test_keys # candidates for skipping

for a in all_pkgs:
    skip_keys -= get_recursive_deps(a).build_keys # don't skip build dependencies

for a in target_tests | downstream_tests:
    skip_keys -= get_recursive_deps(a).test_keys # don't skip test dependencies

ignored_pkgs = set(paths.keys()) - all_pkgs

def export(name, iterable):
    print('export ' + name + '=('+' '.join(('"%s"' % i for i in sorted(iterable)))+')')
    print()

export('upstream_pkgs', upstream_pkgs - target_pkgs) # filter intra-target test dependencies
export('target_pkgs', target_pkgs)
export('downstream_pkgs', downstream_pkgs - upstream_pkgs - target_pkgs) # don't build them again

export('target_tests', target_tests)
export('downstream_tests', downstream_tests - target_tests) # don't run target tests again

export('rosdep_paths', (workspace_src+'/'+paths[p] for p in all_pkgs if p in paths))
export('rosdep_skip_keys', (skip_keys - all_pkgs) | ignored_pkgs)
export('ignored_pkgs', ignored_pkgs)
