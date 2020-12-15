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

if [ ! "$APTKEY_STORE_SKS" ]; then export APTKEY_STORE_SKS="hkp://keyserver.ubuntu.com:80"; fi  # Export a variable for SKS URL for break-testing purpose.
if [ ! "$HASHKEY_SKS" ]; then export HASHKEY_SKS="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"; fi

function  ros1_defaults {
    DEFAULT_OS_CODE_NAME=$1
    export ROS1_DISTRO=${ROS1_DISTRO:-$ROS_DISTRO}
    export BUILDER=${BUILDER:-catkin_tools}
    export ROS_VERSION=1
    export ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION:-2}
}
function  ros2_defaults {
    DEFAULT_OS_CODE_NAME=$1
    export ROS2_DISTRO=${ROS2_DISTRO:-$ROS_DISTRO}
    export BUILDER=${BUILDER:-colcon}
    export ROS_VERSION=2
    export ROS_PYTHON_VERSION=3
}
function use_snapshot() {
    export ROS_REPOSITORY_PATH="http://snapshots.ros.org/${ROS_DISTRO}/$1/ubuntu"
    export HASHKEY_SKS="AD19BAB3CBF125EA"
}

function use_repo_or_final_snapshot() {
    if [ "$ROS_VERSION_EOL" = true ]; then
        use_snapshot final
        if [ -n "$ROS_REPO" ]; then
            ici_warn "'$ROS_DISTRO' is in end-of-life state, ROS_REPO='$ROS_REPO' gets ignored"
        fi
    else
        export ROS_REPOSITORY_PATH="$1"
        if [ "$ROS_REPO" = "ros-shadow-fixed" ]; then
            ici_warn "ROS_REPO='ros-shadow-fixed' was renamed to ROS_REPO='testing'"
        fi
    fi
}
function set_ros_variables {
    case "$ROS_DISTRO" in
    "indigo"|"jade")
        ros1_defaults "trusty"
        export ROS_VERSION_EOL=true
        ;;
    "kinetic")
        ros1_defaults "xenial"
        ;;
    "lunar")
        ros1_defaults "xenial"
        export ROS_VERSION_EOL=true
        ;;
    "melodic")
        ros1_defaults "bionic"
        ;;
    "noetic")
        export BUILDER=${BUILDER:-colcon}
        ros1_defaults "focal"
        export ROS_PYTHON_VERSION=3
        ;;
    "ardent")
        ros2_defaults "xenial"
        export ROS_VERSION_EOL=true
        ;;
    "bouncy"|"crystal")
        ros2_defaults "bionic"
        export ROS_VERSION_EOL=true
        ;;
    "dashing")
        ros2_defaults "bionic"
        ;;
    "eloquent")
        ros2_defaults "bionic"
        ;;
    "foxy")
        ros2_defaults "focal"
        ;;
    "rolling")
        ros2_defaults "focal"
        ;;
    esac

    local prefix=ros
    if [ "$ROS_VERSION" -eq 2 ]; then
      prefix=ros2
    fi

    if [ ! "$ROS_REPOSITORY_PATH" ]; then
        case "${ROS_REPO:-testing}" in
        "building")
            use_repo_or_final_snapshot "http://repositories.ros.org/ubuntu/building/"
            ;;
        "main")
            use_repo_or_final_snapshot "http://packages.ros.org/$prefix/ubuntu"
            ;;
        "ros")
            if [ "$ROS_VERSION" -eq 2 ]; then
                ici_warn "ROS_REPO=ros would select the ROS1 repository, please use ROS_REPO=main"
            fi
            use_repo_or_final_snapshot "http://packages.ros.org/$prefix/ubuntu"
            ;;
        "ros1")
            use_repo_or_final_snapshot "http://packages.ros.org/ros/ubuntu"
            ;;
        "ros2")
            use_repo_or_final_snapshot "http://packages.ros.org/ros2/ubuntu"
            ;;
        "testing")
            use_repo_or_final_snapshot "http://packages.ros.org/$prefix-testing/ubuntu"
            ;;
        "ros-shadow-fixed"|"ros-testing")
            if [ "$ROS_VERSION" -eq 2 ]; then
                ici_warn "ROS_REPO=$ROS_REPO would select the ROS1 repository, please use ROS_REPO=testing"
            fi
            use_repo_or_final_snapshot "http://packages.ros.org/$prefix-testing/ubuntu"
            ;;
        "ros1-testing")
            use_repo_or_final_snapshot "http://packages.ros.org/ros-testing/ubuntu"
            ;;
        "ros2-testing")
            use_repo_or_final_snapshot "http://packages.ros.org/ros2-testing/ubuntu"
            ;;
        "final"|????-??-??)
            use_snapshot "${ROS_REPO}"
            ;;
        *)
            ici_error "ROS repo '$ROS_REPO' is not supported"
            ;;
        esac
    fi
}

# exit with error if OS_NAME is set, but OS_CODE_NAME is not.
# assume ubuntu as default
if [ -z "$OS_NAME" ]; then
    export OS_NAME=ubuntu
elif [ -z "$OS_CODE_NAME" ]; then
    ici_error "please specify OS_CODE_NAME"
fi

if [ -n "$UBUNTU_OS_CODE_NAME" ]; then # for backward-compatibility
    export OS_CODE_NAME=$UBUNTU_OS_CODE_NAME
fi

if [ -z "$OS_CODE_NAME" ]; then
    case "$ROS_DISTRO" in
    "")
        if [ -n "$DOCKER_IMAGE" ]; then
          # try to reed ROS_DISTRO from (base) image
          ici_docker_try_pull "${DOCKER_IMAGE}"
          export ROS_DISTRO
          ROS_DISTRO=$(docker image inspect --format "{{.Config.Env}}" "${DOCKER_IMAGE}" | grep -o -P "(?<=ROS_DISTRO=)[a-z]*") || true
        fi
        if [ -z "$ROS_DISTRO" ]; then
            ici_error "Please specify ROS_DISTRO"
        fi
        set_ros_variables
        ;;
    *)
        set_ros_variables
        if [ -z "$DEFAULT_OS_CODE_NAME" ]; then
            ici_error "ROS distro '$ROS_DISTRO' is not supported"
        fi
        export OS_CODE_NAME=$DEFAULT_OS_CODE_NAME
        ;;
    esac
else
    set_ros_variables
fi
