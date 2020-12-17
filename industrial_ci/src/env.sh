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

export ROS_DISTRO=${ROS_DISTRO:-}
export ROS_REPOSITORY_PATH=${ROS_REPOSITORY_PATH:-}
export ROS_REPO=${ROS_REPO:-testing}

export DEBUG_BASH=${DEBUG_BASH:-false}
export TEST=${TEST:-}

export EXPECT_EXIT_CODE=${EXPECT_EXIT_CODE:-0}
export APTKEY_STORE_SKS=${APTKEY_STORE_SKS:-hkp://keyserver.ubuntu.com:80}
export APTKEY_STORE_HTTPS=${APTKEY_STORE_HTTPS:-}
export HASHKEY_SKS=${HASHKEY_SKS:-C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654}
export ADDITIONAL_DEBS=${ADDITIONAL_DEBS:-}

export ABICHECK_URL=${ABICHECK_URL:-}
export ABICHECK_VERSION=${ABICHECK_VERSION:-}

export BLACK_CHECK=${BLACK_CHECK:-false}
export BUILDER=${BUILDER:-}

export CATKIN_LINT=${CATKIN_LINT:-false}
export CATKIN_LINT_ARGS=${CATKIN_LINT_ARGS:-}

export CLANG_FORMAT_CHECK=${CLANG_FORMAT_CHECK:-}
export CLANG_FORMAT_VERSION=${CLANG_FORMAT_VERSION:-}

export CLANG_TIDY=${CLANG_TIDY:-false}
export CLANG_TIDY_ARGS=${CLANG_TIDY_ARGS:-}
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
export OS_NAME=${OS_NAME:-ubuntu}

export ROSDEP_SKIP_KEYS=${ROSDEP_SKIP_KEYS:-}

export TARGET_CMAKE_ARGS=${TARGET_CMAKE_ARGS:-}

export UPSTREAM_CMAKE_ARGS=${UPSTREAM_CMAKE_ARGS:-}
export UPSTREAM_WORKSPACE=${UPSTREAM_WORKSPACE:-}

