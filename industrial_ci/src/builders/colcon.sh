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

_colcon_event_handlers=(desktop_notification- status- terminal_title-)

function builder_setup {
    ici_install_pkgs_for_command colcon python3-colcon-common-extensions
    if [ "$ROS_DISTRO" = "kinetic" ]; then
        ici_install_pkgs_for_command pip3 python3-pip
        ici_asroot pip3 install -U setuptools
    fi
}

function builder_run_build {
    local extend=$1; shift
    local ws=$1; shift
    ici_exec_in_workspace "$extend" "$ws" colcon build --event-handlers "${_colcon_event_handlers[@]}" "$@"
}

function builder_run_tests {
    local extend=$1; shift
    local ws=$1; shift
    local output_handler
    if [ "$IMMEDIATE_TEST_OUTPUT" == true ]; then
        output_handler="console_direct+"
    else
        output_handler="console_cohesion+"
    fi
    ici_exec_in_workspace "$extend" "$ws" colcon test --event-handlers "${_colcon_event_handlers[@]}" "${output_handler}"
}

function builder_test_results {
    local extend=$1; shift
    local ws=$1; shift
    ici_exec_in_workspace "$extend" "$ws" colcon test-result --verbose
}
