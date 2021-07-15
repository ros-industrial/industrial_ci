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

# shellcheck source=industrial_ci/src/builders/catkin_make_isolated.sh
source "${ICI_SRC_PATH}/builders/catkin_make_isolated.sh"

ici_warn "BUILDER=catkin_make_isolated_devel should only be used in addition to the other non-devel builders"

function ici_extend_space {
    echo "$1/devel"
}

function builder_run_build {
    local opts=()
    _append_job_opts opts PARALLEL_BUILDS 0
    _run_catkin_make_isolated all "${opts[@]}" "$@"
}
