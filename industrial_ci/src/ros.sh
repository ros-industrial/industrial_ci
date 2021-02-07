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

function _ros1_defaults() {
    export OS_CODE_NAME=${OS_CODE_NAME:-$1}
    export ROS1_DISTRO=${ROS1_DISTRO:-$ROS_DISTRO}
    export BUILDER=${BUILDER:-catkin_tools}
    export ROS_VERSION=1
    export ROS_VERSION_EOL=false
    export ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION:-2}
}

function _ros2_defaults() {
    export OS_CODE_NAME=${OS_CODE_NAME:-$1}
    export ROS2_DISTRO=${ROS2_DISTRO:-$ROS_DISTRO}
    export BUILDER=${BUILDER:-colcon}
    export ROS_VERSION=2
    export ROS_VERSION_EOL=false
    export ROS_PYTHON_VERSION=3
}

function _set_ros_defaults() {
    case "$ROS_DISTRO" in
    "indigo" | "jade")
        _ros1_defaults "trusty"
        export ROS_VERSION_EOL=true
        ;;
    "kinetic")
        _ros1_defaults "xenial"
        ;;
    "lunar")
        _ros1_defaults "xenial"
        export ROS_VERSION_EOL=true
        ;;
    "melodic")
        _ros1_defaults "bionic"
        ;;
    "noetic")
        export BUILDER=${BUILDER:-colcon}
        _ros1_defaults "focal"
        export ROS_PYTHON_VERSION=3
        ;;
    "ardent")
        _ros2_defaults "xenial"
        export ROS_VERSION_EOL=true
        ;;
    "bouncy" | "crystal")
        _ros2_defaults "bionic"
        export ROS_VERSION_EOL=true
        ;;
    "dashing")
        _ros2_defaults "bionic"
        ;;
    "eloquent")
        _ros2_defaults "bionic"
        ;;
    "foxy")
        _ros2_defaults "focal"
        ;;
    "rolling")
        _ros2_defaults "focal"
        ;;
    "false")
        unset ROS_DISTRO
        ;;
    *)
        ici_error "ROS_DISTRO '$ROS_DISTRO' is not supported"
        ;;
    esac

    if [ "$ROS_PYTHON_VERSION" = 2 ]; then
        export PYTHON_VERSION_NAME=python
    elif [ "$ROS_PYTHON_VERSION" = 3 ]; then
        export PYTHON_VERSION_NAME=python3
    fi

}

function _use_snapshot() {
    export ROS_REPOSITORY_PATH="http://snapshots.ros.org/${ROS_DISTRO?ROS_DISTRO needs to be set}/$1/ubuntu"
    export HASHKEY_SKS="AD19BAB3CBF125EA"
}

function _use_repo_or_final_snapshot() {
    if [ "$ROS_VERSION_EOL" = true ]; then
        _use_snapshot final
        if [ "$ROS_REPO" != "testing" ]; then
            ici_warn "'$ROS_DISTRO' is in end-of-life state, ROS_REPO='$ROS_REPO' gets ignored"
        fi
    else
        export ROS_REPOSITORY_PATH="$1"
        if [ "${ROS_REPO}" = "ros-shadow-fixed" ]; then
            ici_warn "ROS_REPO='ros-shadow-fixed' was renamed to ROS_REPO='testing'"
        fi
    fi
}

function _get_prefix() {
    if [ "$ROS_VERSION" -eq 2 ]; then
        echo ros2
    else
        echo ros
    fi
}

function _set_ros_package_path() {
    if [ -z "${ROS_REPOSITORY_PATH}" ]; then
        case "$ROS_REPO" in
        "building")
            _use_repo_or_final_snapshot "http://repositories.ros.org/ubuntu/building/"
            ;;
        "main")
            _use_repo_or_final_snapshot "http://packages.ros.org/$(_get_prefix)/ubuntu"
            ;;
        "ros")
            if [ "$ROS_VERSION" -eq 2 ]; then
                ici_warn "ROS_REPO=ros would select the ROS1 repository, please use ROS_REPO=main"
            fi
            _use_repo_or_final_snapshot "http://packages.ros.org/$(_get_prefix)/ubuntu"
            ;;
        "ros1")
            _use_repo_or_final_snapshot "http://packages.ros.org/ros/ubuntu"
            ;;
        "ros2")
            _use_repo_or_final_snapshot "http://packages.ros.org/ros2/ubuntu"
            ;;
        "testing")
            _use_repo_or_final_snapshot "http://packages.ros.org/$(_get_prefix)-testing/ubuntu"
            ;;
        "ros-shadow-fixed" | "ros-testing")
            if [ "$ROS_VERSION" -eq 2 ]; then
                ici_warn "ROS_REPO=$ROS_REPO would select the ROS1 repository, please use ROS_REPO=testing"
            fi
            _use_repo_or_final_snapshot "http://packages.ros.org/$(_get_prefix)-testing/ubuntu"
            ;;
        "ros1-testing")
            _use_repo_or_final_snapshot "http://packages.ros.org/ros-testing/ubuntu"
            ;;
        "ros2-testing")
            _use_repo_or_final_snapshot "http://packages.ros.org/ros2-testing/ubuntu"
            ;;
        "final" | ????-??-??)
            _use_snapshot "${ROS_REPO}"
            ;;
        *)
            ici_error "ROS repo '$ROS_REPO' is not supported"
            ;;
        esac
    fi
}

function ici_configure_ros() {
    if [ -n "${ROS_DISTRO}" ]; then
        _set_ros_defaults
        _set_ros_package_path
    fi
}
