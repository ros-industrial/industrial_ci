#!/bin/bash

# Copyright (c) 2020, Mathias Lüdtke
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
[[ "${BASH_VERSINFO[0]}_${BASH_VERSINFO[1]}" < "4_4" ]] || set -u
_CLEANUP=${_CLEANUP-}

# shellcheck source=industrial_ci/src/env.sh
source "${ICI_SRC_PATH}/env.sh"

# shellcheck source=industrial_ci/src/util.sh
source "${ICI_SRC_PATH}/util.sh"

# shellcheck source=industrial_ci/src/ros.sh
source "${ICI_SRC_PATH}/ros.sh"

# shellcheck source=industrial_ci/src/workspace.sh
source "${ICI_SRC_PATH}/workspace.sh"

ici_setup

if [ "$DEBUG_BASH" = true ]; then set -x; fi # print trace if DEBUG

ici_configure_ros

export TARGET_WORKSPACE=${TARGET_WORKSPACE:-$TARGET_REPO_PATH}
export BASEDIR=${BASEDIR:-$HOME}

export LANG=${LANG:-C.UTF-8}
export LC_ALL=${LC_ALL:-C.UTF-8}
export TERM=${TERM:-dumb}

if [ -z "${CC:-}" ]; then unset CC; fi
if [ -z "${CFLAGS:-}" ]; then unset CFLAGS; fi
if [ -z "${CPPFLAGS:-}" ]; then unset CPPFLAGS; fi
if [ -z "${CXX:-}" ]; then unset CXX; fi
if [ -z "${CXXFLAGS:-}" ]; then unset CXXLAGS; fi

TEST=$1; shift
ici_source_component TEST tests

ici_step "init" ici_init_apt

if [ -n "${UNDERLAY:-}" ]; then
    if [ ! -f "$UNDERLAY/setup.bash" ] && [ "$UNDERLAY" != "/opt/ros/$ROS_DISTRO" ]; then
        ici_error "UNDERLAY '$UNDERLAY' does not contain a setup.bash"
    fi
else
    if [ -n "${ROS_DISTRO:-}" ]; then
        export UNDERLAY=${UNDERLAY:-/opt/ros/$ROS_DISTRO}
    fi
fi

"$@" || ici_exit

ici_hook "after_script" || ici_exit

ici_exit 0
