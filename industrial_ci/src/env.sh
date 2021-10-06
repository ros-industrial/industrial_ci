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

export PYTHONUNBUFFERED=${PYTHONUNBUFFERED-1}

export ROS_DISTRO=${ROS_DISTRO:-}
export ROS_REPOSITORY_PATH=${ROS_REPOSITORY_PATH:-}
export ROS_REPO=${ROS_REPO:-}
export ROS_REPOSITORY_KEY=${ROS_REPOSITORY_KEY:-"$ICI_SRC_PATH/keys/ros.asc"}

export DEBUG_BASH=${DEBUG_BASH:-false}
export TEST=${TEST:-}

export EXPECT_EXIT_CODE=${EXPECT_EXIT_CODE:-0}
export APTKEY_STORE_SKS=${APTKEY_STORE_SKS:-hkp://keyserver.ubuntu.com:80}
export APTKEY_STORE_HTTPS=${APTKEY_STORE_HTTPS:-}
export HASHKEY_SKS=${HASHKEY_SKS:-}
export ADDITIONAL_DEBS=${ADDITIONAL_DEBS:-}

export ABICHECK_URL=${ABICHECK_URL:-}
export ABICHECK_VERSION=${ABICHECK_VERSION:-}
export ABICHECK_MERGE=${ABICHECK_MERGE:-false}

export APT_PROXY=${APT_PROXY:-}

export BLACK_CHECK=${BLACK_CHECK:-false}
export BUILDER=${BUILDER:-}

export CATKIN_LINT=${CATKIN_LINT:-false}
export CATKIN_LINT_ARGS=${CATKIN_LINT_ARGS:-}

export PYLINT_ARGS=${PYLINT_ARGS:-}
export PYLINT_CHECK=${PYLINT_CHECK:-false}
export PYLINT_EXCLUDE=${PYLINT_EXCLUDE:-}

export CLANG_FORMAT_CHECK=${CLANG_FORMAT_CHECK:-}
export CLANG_FORMAT_VERSION=${CLANG_FORMAT_VERSION:-}

export CLANG_TIDY=${CLANG_TIDY:-false}
export CLANG_TIDY_ARGS=${CLANG_TIDY_ARGS:-}
export CLANG_TIDY_BASE_REF=${CLANG_TIDY_BASE_REF:-}
export CLANG_TIDY_JOBS=${CLANG_TIDY_JOBS:-}

export CCACHE_DIR=${CCACHE_DIR:-}

export CMAKE_ARGS=${CMAKE_ARGS:-}

export DOWNSTREAM_CMAKE_ARGS=${DOWNSTREAM_CMAKE_ARGS:-}
export DOWNSTREAM_WORKSPACE=${DOWNSTREAM_WORKSPACE:-}

export IMMEDIATE_TEST_OUTPUT=${IMMEDIATE_TEST_OUTPUT:-false}
export NOT_TEST_BUILD=${NOT_TEST_BUILD:-false}
export NOT_TEST_DOWNSTREAM=${NOT_TEST_DOWNSTREAM:-false}
export PARALLEL_BUILDS=${PARALLEL_BUILDS:-0}
export PARALLEL_TESTS=${PARALLEL_TESTS:-1}

export PRERELEASE=${PRERELEASE:-false}

case "${OS_CODE_NAME-}" in
# https://wiki.debian.org/DebianReleases#Production_Releases
"jessie"|"stretch"|"buster"|"bullseye"|"bookwork"|"trixie")
    export OS_NAME=debian
    ;;
*)
    export OS_NAME=${OS_NAME:-ubuntu}
    ;;
esac

export ROSDEP_SKIP_KEYS=${ROSDEP_SKIP_KEYS:-}

export TARGET_CMAKE_ARGS=${TARGET_CMAKE_ARGS:-}

export UPSTREAM_CMAKE_ARGS=${UPSTREAM_CMAKE_ARGS:-}
export UPSTREAM_WORKSPACE=${UPSTREAM_WORKSPACE:-}

export PREFIX=${PREFIX:-}
