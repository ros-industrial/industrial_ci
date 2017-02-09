#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Lüdtke
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

if [ ! "$CATKIN_PARALLEL_JOBS" ]; then export CATKIN_PARALLEL_JOBS="-p4"; fi
if [ ! "$CATKIN_PARALLEL_TEST_JOBS" ]; then export CATKIN_PARALLEL_TEST_JOBS="$CATKIN_PARALLEL_JOBS"; fi
if [ ! "$ROS_PARALLEL_JOBS" ]; then export ROS_PARALLEL_JOBS="-j8"; fi
if [ ! "$ROS_PARALLEL_TEST_JOBS" ]; then export ROS_PARALLEL_TEST_JOBS="$ROS_PARALLEL_JOBS"; fi
# If not specified, use ROS Shadow repository http://wiki.ros.org/ShadowRepository
if [ ! "$ROS_REPOSITORY_PATH" ]; then export ROS_REPOSITORY_PATH="http://packages.ros.org/ros-shadow-fixed/ubuntu"; fi
# .rosintall file name
if [ ! "$ROSINSTALL_FILENAME" ]; then export ROSINSTALL_FILENAME=".travis.rosinstall"; fi
# For apt key stores
if [ ! "$APTKEY_STORE_HTTPS" ]; then export APTKEY_STORE_HTTPS="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"; fi
if [ ! "$APTKEY_STORE_SKS" ]; then export APTKEY_STORE_SKS="hkp://ha.pool.sks-keyservers.net"; fi  # Export a variable for SKS URL for break-testing purpose.
if [ ! "$HASHKEY_SKS" ]; then export HASHKEY_SKS="0xB01FA116"; fi
if [ "$USE_DEB" ]; then  # USE_DEB is deprecated. See https://github.com/ros-industrial/industrial_ci/pull/47#discussion_r64882878 for the discussion.
    if [ "$USE_DEB" != "true" ]; then export UPSTREAM_WORKSPACE="file";
    else export UPSTREAM_WORKSPACE="debian";
    fi
fi
if [ ! "$UPSTREAM_WORKSPACE" ]; then export UPSTREAM_WORKSPACE="debian"; fi

export UBUNTU_OS_CODE_NAME
if [ -z "$UBUNTU_OS_CODE_NAME" ]; then
    case "$ROS_DISTRO" in
    "hydro")
        UBUNTU_OS_CODE_NAME="precise"
        ;;
    "indigo"|"jade")
        UBUNTU_OS_CODE_NAME="trusty"
        ;;
    "kinetic")
        UBUNTU_OS_CODE_NAME="xenial"
        ;;
    *)
        error "ROS distro '$ROS_DISTRO' is not supported"
        ;;
esac
fi
