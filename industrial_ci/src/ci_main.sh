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
export ICI_SRC_PATH=$(pwd)  # The path on CI service (e.g. Travis CI) to industrial_ci src dir.

source ${ICI_SRC_PATH}/util.sh
source ${ICI_SRC_PATH}/env.sh
source ${ICI_SRC_PATH}/docker.sh

trap ici_exit EXIT # install industrial_ci exit handler

# Start prerelease, and once it finishs then finish this script too.
if [ "$PRERELEASE" == true ]; then
  source ${ICI_SRC_PATH}/tests/ros_prerelease.sh
  run_ros_prerelease
else
  source ${ICI_SRC_PATH}/tests/source_tests.sh
fi

ici_time_start after_script

  cd $TARGET_REPO_PATH
  if [ "${AFTER_SCRIPT// }" != "" ]; then sh -e -c "${AFTER_SCRIPT}"; fi

ici_time_end  # after_script

cd $TARGET_REPO_PATH  # cd back to the repository's home directory with travis

ici_exit 0
