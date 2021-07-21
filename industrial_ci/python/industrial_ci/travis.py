#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2017, Mathias Lüdtke
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import difflib
import os
import os.path
import re
import subprocess
import sys

import yaml

# find config file either in first argument or curret working directory
def find_config_file(args, names):
    def test_file(dir): # test if one of the file names is present
        for n in  names:
            p = os.path.join(dir, n)
            if os.path.isfile(p):
                return os.path.abspath(p)
        return None

    if len(args) >  0: # test first argument if available
        arg0=args[0]
        if os.path.isfile(arg0):
            return os.path.abspath(arg0),args[1:]
        p = test_file(arg0)
        if p is not None:
            return p, args[1:]
    return test_file(os.getcwd()), args

# parse range tuples from arguments
def parse_ranges(args, offset=0):
    r = []
    def apply_offset(i): # adjust for user offsets
        ao = int(i) + offset
        if ao < 0:
            raise ValueError("%s is not in supported range" % str(i))
        return ao

    for a in args:
        if '-' in a: # range string
            p1, p2 = a.split('-', 2)
            if p1 == '' and len(r) == 0: # -X
                p1 = 0
            else: # X-[Y]
                p1 = apply_offset(p1)
            if p2 == '': # [X]-
                r.append((p1, -1))
                break
            r.append((p1, apply_offset(p2)+1)) # X-Y
        else:
            i = apply_offset(a)
            r.append((i, i+1))
    return r

def apply_ranges(ranges, num):
    for start,end  in ranges:
        if end == -1:
            end = num
        for i in range(start,end):
            yield i


def read_yaml(p):
    with open(p) as f:
        return yaml.safe_load(f)

# read global and job-specific envs from
def read_env(env):
    m = env
    g = ''
    if isinstance(env, dict):
        m = env['matrix']
        if 'global' in env:
            g = ' '.join(env['global'])
    return g, m

def read_allow_failures(config):
    try:
        af =  config['matrix']['allow_failures']
    except:
        return list()
    return list(x['env'] for x in af)

def read_num_include(config):
    try:
        return len(config['matrix']['include'])
    except:
        return 0

def parse_extra_args(args):
    try:
        extra = args.index('--')
        return args[0:extra], args[extra+1:]
    except ValueError:
        return args, []

env_assigment = re.compile(r"[a-zA-Z_][a-zA-Z0-9_]*=")
def gen_env(e):
    if env_assigment.match(e):
        return e
    return '%s=%s' % (e, os.getenv(e, ''))

# from https://github.com/travis-ci/travis-build/blob/73bf69a439bb546520a5e5b6b6847fb5424a7c9f/lib/travis/build/env/var.rb#L5
travis_env = re.compile(r"([\w]+)=((?:[^\"'`\ ]?(\"|'|`).*?((?<!\\)\3))+|(?:[^\$]?\$\(.*?\))+|[^\"'\ ]+|(?=\s)|\Z)")
def filter_env(e):
    return ' '.join(map(lambda m: m.group(0), travis_env.finditer(e)))

def highlight_diff(e, n=''):
    f = filter_env(e)
    def mark(d):
        tag, chunk = d[0:2], d[2:]
        return chunk if tag == "  " or not chunk.strip(' ') else "\033[1;41m%s\033[1;%sm" % (chunk, str(n))
    return ''.join(mark(d) for d in difflib.ndiff(e, f, None, None))

def print_help(cmd):
    print("""
Usage: %s [PATH] [RANGE*] [-- [ENV[=VALUE]*]]

Parses the travis config in the given path or in the current working directory and runs all specified tests sequentially
If no range is given, the list of jobs will get printed.

The number of tests can be reduced by specifying one or more ranges:
* single job: 1 (only first)
* range: 2-3 (only second and third)
* open start, only as first range: -4 (jobs 1 to 4)
* open end, only as last range: 7- (job 7 and all following jobs)
* open range: - (all jobs)

Complex examples for a matrix wih 12 jobs:

* -4 7 8: runs jobs 1 2 3 4 7 8
* 1 7-9: runs jobs 1 7 8 9
* 1 7-9 11-: runs jobs 1 7 8 9 11 12
* -: runs all jobs

The jobs will be run in clean environments.
Only DOCKER_PORT, SSH_AUTH_SOCK, and TERM will be kept.
Additional variable names can be passed at the end.

""" % cmd)

def main(scripts_dir, argv):
    if '--help' in argv:
        print_help(argv[0])
        sys.exit(0)

    args, extra_env = parse_extra_args(argv[1:])

    path, args = find_config_file(args, ['.travis.yml', '.travis.yaml'])
    config = read_yaml(path)
    global_env, job_envs = read_env(config['env'])
    allow_failures = read_allow_failures(config)
    job_envs =   [ x for x in job_envs if x not in allow_failures ] \
               + [None] * read_num_include(config) \
               + [ x for x in job_envs if x in allow_failures ]

    if len(args) == 0:
        if(len(global_env) > 0):
            print('Globals: %s' % str(highlight_diff(global_env)))
        jobs = len(job_envs)
        digits = len(str(jobs))
        for i in range(jobs):
            print('Job %s%s: %s' % ( str(i+1).rjust(digits),
                                     ' (allow_failures)' if job_envs[i] in allow_failures else '',
                                     highlight_diff(job_envs[i]) if job_envs[i] is not None else "<unsupported job from 'include' section>"))
        print("run all with %s -" % sys.argv[0])
        sys.exit(0)

    ranges = parse_ranges(args, -1)

    run_ci = [os.path.join(scripts_dir, "run_ci"), os.path.dirname(path), filter_env(global_env)]
    run_ci_diff = [os.path.join(scripts_dir, "run_ci"), os.path.dirname(path), highlight_diff(global_env, 44)]

    bash = ['env', '-i'] +list(map(gen_env, ['DOCKER_PORT', 'HOME', 'PATH', 'SSH_AUTH_SOCK', 'TERM'])) + ['bash','-e']

    selection = set(apply_ranges(ranges, len(job_envs)))

    for i in selection:
        if job_envs[i] is None:
            print("\033[1;43mSkipped job %d, because jobs from 'include' section are not supported\033[1;m" %(i+1,))
            continue
        cmd = ' '.join(run_ci + [filter_env(job_envs[i])] + list(map(gen_env, extra_env)))
        cmd_diff = ' '.join(run_ci_diff + [highlight_diff(job_envs[i], 44)] + list(map(gen_env, extra_env)))
        print('\033[1;44mRunning job %d%s: %s\033[1;m' %(i+1, ' (allow_failures)' if job_envs[i] in allow_failures else '', cmd_diff))

        proc = subprocess.Popen(bash, stdin=subprocess.PIPE)
        proc.communicate(cmd.encode())
        if proc.returncode:
            print('\033[1;41mFailed job %d: %s\033[1;m' %(i+1, cmd))
            if job_envs[i] not in allow_failures:
                sys.exit(proc.returncode)

