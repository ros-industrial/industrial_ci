#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
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

## Greatly inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis

## This is a "common" script that can be run on travis CI at a downstream github repository.
## See ./README.rst for the detailed usage.

set -e # exit script on errors
if [ "$DEBUG_BASH" ]; then set -x; fi # print trace if DEBUG

# Define some env vars that need to come earlier than util.sh
export ICI_SRC_PATH; ICI_SRC_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"  # The path on CI service (e.g. Travis CI) to industrial_ci src dir.

# shellcheck source=industrial_ci/src/util.sh
source "${ICI_SRC_PATH}/util.sh"
# shellcheck source=industrial_ci/src/docker.sh
source "${ICI_SRC_PATH}/docker.sh"
# shellcheck source=industrial_ci/src/workspace.sh
source "${ICI_SRC_PATH}/workspace.sh"
# shellcheck source=industrial_ci/src/env.sh
source "${ICI_SRC_PATH}/env.sh"

trap ici_exit EXIT # install industrial_ci exit handler

# Start prerelease, and once it finishs then finish this script too.
if [ "$PRERELEASE" == true ]; then
  # shellcheck source=industrial_ci/src//tests/ros_prerelease.sh
  source "${ICI_SRC_PATH}/tests/ros_prerelease.sh"
  run_ros_prerelease
elif [ -n "$ABICHECK_URL" ]; then
  # shellcheck source=industrial_ci/src/tests/abi_check.sh
  source "${ICI_SRC_PATH}/tests/abi_check.sh"
  run_abi_check
elif [ -n "$CLANG_FORMAT_CHECK" ]; then
  # shellcheck source=industrial_ci/src/tests/clang_format_check.sh
  source "${ICI_SRC_PATH}/tests/clang_format_check.sh"
  run_clang_format_check
else
  # shellcheck source=industrial_ci/src/tests/source_tests.sh
  source "${ICI_SRC_PATH}/tests/source_tests.sh"
fi

ici_hook "after_script"

ici_exit 0
