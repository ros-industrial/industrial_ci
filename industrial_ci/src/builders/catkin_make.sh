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
    if [ "$jobs" -gt 0 ]; then
        _append_job_opts_res+=("-j$jobs" "-l$jobs")
    fi
}

function builder_setup {
    ici_install_pkgs_for_command catkin_make "ros-${ROS_DISTRO}-catkin"
}

function builder_run_build {
    local extend=$1; shift
    local ws=$1; shift
    local opts=()
    _append_job_opts opts PARALLEL_BUILDS 0
    ici_cmd ici_exec_in_workspace "$extend" "$ws" catkin_make --make-args install "${opts[@]}" "$@"
}

function builder_run_tests {
    local extend=$1; shift
    local ws=$1; shift
    local opts=()
    _append_job_opts opts PARALLEL_TESTS 1
    ici_cmd ici_exec_in_workspace "$extend" "$ws" catkin_make --make-args run_tests "${opts[@]}" "$@"
}

function builder_test_results {
    local extend=$1; shift
    local ws=$1; shift
    ici_cmd ici_exec_in_workspace "$extend" "$ws" catkin_test_results --verbose
}
