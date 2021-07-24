#!/bin/bash

# Copyright (c) 2021, Mathias Lüdtke
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

# shellcheck source=industrial_ci/src/builders/catkin_tools.sh
source "${ICI_SRC_PATH}/builders/catkin_tools.sh"

function _catkin_config {
    local extend=$1; shift
    local ws=$1; shift
    ici_cmd ici_exec_in_workspace "$extend" "$ws" catkin config --install --isolate-devel
}
