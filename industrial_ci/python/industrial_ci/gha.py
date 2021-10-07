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

from industrial_ci.common import *

# read global and job-specific envs from
def read_env(env):
    m = []
    g = ''
    if isinstance(env, dict):
        for l in env['strategy']['matrix']['include']:
            job_env = ''
            for key, value in l.items():
                job_env += key+"='"+str(value) + "' " # adds a space to every element
            job_env = job_env[:-1] # remove space from last element
            m.append(job_env)
        for key, value in env['env'].items():
            g += " "+key+"='"+str(value)+"'"
    return g, m

def main(scripts_dir, argv):
    if '--help' in argv:
        print_help(argv[0])
        sys.exit(0)

    args, extra_env = parse_extra_args(argv[1:])

    path, args = find_config_file(args, ['.github/workflows/main.yml','.github/workflows/main.yaml'])
    config = read_yaml(path)
    global_env, job_envs = read_env(config['jobs']['industrial_ci'])
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

