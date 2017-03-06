#!/usr/bin/env python
from __future__ import print_function
from os import getenv, path
from catkin_pkg.packages import find_packages
from itertools import chain
from sys import stderr

# read environment variables

workspace = getenv("CATKIN_WORKSPACE")
if workspace is None:
    print("please set $CATKIN_WORKSPACE", file=stderr)
    exit(1)
workspace_src=path.join(workspace, 'src')

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

# dependencies of workspace packages within workspace
workspace_deps = dict((p.name, set(d.name for d in chain(p.build_depends, p.buildtool_depends, p.buildtool_export_depends, p.exec_depends, p.build_export_depends, p.test_depends)) & workspace_pkgs) for p in
 known_pkgs.values())

recursive_deps = dict() # recursive dependencies in workspace
def get_recursive_deps(name):
    global recursive_deps, workspace_deps
    try:
        return recursive_deps[name]
    except KeyError:
        deps = workspace_deps[name]
        recursive_deps[name] = deps = reduce(lambda s,n: s.union(get_recursive_deps(n)), deps, deps) # recursive_deps = workspace_deps + recursive_workspace_deps
        return deps

upstream_pkgs = set()
downstream_tests = set(downstream_pkgs)

# add all recursive dependencies of pre-configured dependencies
downstream_pkgs = reduce(lambda s,n: s.union(get_recursive_deps(n)), downstream_pkgs, downstream_pkgs)

for p in workspace_pkgs:
    deps = get_recursive_deps(p)
    if p in target_pkgs: # fill recursive dependencies of target packages
        upstream_pkgs |= deps
    if len(deps & target_pkgs) > 0: # is downstream packages
        if p in pkgs_whitelisted or ( not pkgs_whitelisted and not p in pkgs_blacklisted): # match white and blacklist
            downstream_pkgs |= deps
            downstream_pkgs.add(p)
            downstream_tests.add(p)
    elif p in pkgs_whitelisted: # no recursive dependency, but whitelisted
        upstream_pkgs |= deps
        upstream_pkgs.add(p)

downstream_pkgs -= upstream_pkgs # don't build upstream packages again
downstream_pkgs -= target_pkgs # don't build target packages again

def export(name, iterable):
    print('export ' + name + '=('+' '.join(('"%s"' % i for i in iterable))+')')

export('upstream_pkgs', upstream_pkgs)
export('target_pkgs', target_pkgs)
export('downstream_pkgs', downstream_pkgs)

target_tests = target_pkgs - tests_blacklisted
export('target_tests', target_tests)
export('downstream_tests', downstream_tests - target_tests - tests_blacklisted) # don't run target tests again

all_pkgs = upstream_pkgs | downstream_pkgs | target_pkgs
export('rosdep_paths', (workspace_src+'/'+paths[p] for p in all_pkgs if p in paths))
export('ignored_pkgs', set(paths.keys()) - all_pkgs)
