#!/bin/bash

# Copyright (c) 2019, Mathias Lüdtke
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

function _append_job_opts() {
    local -n _append_job_opts_res=$1
    local jobs
    ici_parse_jobs jobs "$2" "$3"
    if [ "$jobs" -eq 1 ]; then
        _append_job_opts_res+=("-j1" "-p1")
    elif [ "$jobs" -gt 1 ]; then
        _append_job_opts_res+=("-j$jobs")
    fi
}

function builder_setup {
  ici_install_pkgs_for_command catkin "${PYTHON_VERSION_NAME}-catkin-tools" "ros-$ROS_DISTRO-catkin"
}

function builder_run_build {
    local extend=$1; shift
    local ws=$1; shift
    local -a opts
    if [ "${VERBOSE_OUTPUT:-false}" != false ]; then
        opts+=("-vi")
    fi
    _append_job_opts opts PARALLEL_BUILDS 0
    ici_exec_in_workspace "$extend" "$ws" catkin config --install
    ici_exec_in_workspace "$extend" "$ws" catkin build "${opts[@]}" --summarize  --no-status "$@"
}

function builder_run_tests {
    local extend=$1; shift
    local ws=$1; shift
    local -a opts
    _append_job_opts opts PARALLEL_TESTS 1
    ici_exec_in_workspace "$extend" "$ws" catkin build --catkin-make-args run_tests -- "${opts[@]}" --no-status
}

function builder_test_results {
    local extend=$1; shift
    local ws=$1; shift
    ici_exec_in_workspace "$extend" "$ws" catkin_test_results --verbose
}
