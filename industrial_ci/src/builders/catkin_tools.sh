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

function builder_setup {
  ici_install_pkgs_for_command catkin "${PYTHON_VERSION_NAME}-catkin-tools"
}

function builder_run_build {
    local extend=$1; shift
    local ws=$1; shift
    local -a opts
    if ici_is_true"$VERBOSE_OUTPUT"; then
        opts+=("-vi")
    fi
    ici_exec_in_workspace "$extend" "$ws" catkin config --install
    ici_exec_in_workspace "$extend" "$ws" catkin build "${opts[@]}" --summarize  --no-status "$@"
}

function builder_run_tests {
    local extend=$1; shift
    local ws=$1; shift
    local -a opts
    if ici_is_true"$VERBOSE_TESTS"; then
        opts+=(-v)
    fi
    if ici_is_true"$IMMEDIATE_TEST_OUTPUT"; then
        opts+=(-i)
    fi
    if ici_is_false_or_unset "$PARALLEL_TESTS"; then
        opts+=(-j1 -p1)
    fi
    ici_exec_in_workspace "$extend" "$ws" catkin build --catkin-make-args run_tests -- "${opts[@]}" --no-status
}

function builder_test_results {
    local extend=$1; shift
    local ws=$1; shift
    ici_exec_in_workspace "$extend" "$ws" catkin_test_results --verbose
}
