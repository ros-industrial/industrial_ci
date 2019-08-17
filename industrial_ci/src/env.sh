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

ici_enforce_deprecated BEFORE_SCRIPT "Please migrate to new hook system."
ici_enforce_deprecated CATKIN_CONFIG "Explicit catkin configuration is not available anymore."

if [ -n "$NOT_TEST_INSTALL" ]; then
    if [ "$NOT_TEST_INSTALL" != true ]; then
        ici_enforce_deprecated NOT_TEST_INSTALL "testing installed test files has been removed."
    else
        ici_mark_deprecated NOT_TEST_INSTALL "testing installed test files has been removed, NOT_TEST_INSTALL=false is superfluous"
    fi
fi

for v in BUILD_PKGS_WHITELIST PKGS_DOWNSTREAM TARGET_PKGS USE_MOCKUP; do
    ici_enforce_deprecated "$v" "Please migrate to new workspace definition"
done

for v in CATKIN_PARALLEL_JOBS CATKIN_PARALLEL_TEST_JOBS ROS_PARALLEL_JOBS ROS_PARALLEL_TEST_JOBS; do
    ici_mark_deprecated "$v" "Job control is not available anymore"
done

ici_mark_deprecated UBUNTU_OS_CODE_NAME "Was renamed to OS_CODE_NAME."
if [ ! "$APTKEY_STORE_SKS" ]; then export APTKEY_STORE_SKS="hkp://keyserver.ubuntu.com:80"; fi  # Export a variable for SKS URL for break-testing purpose.
if [ ! "$HASHKEY_SKS" ]; then export HASHKEY_SKS="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"; fi

# variables in docker.env without default will be exported with empty string
# this might break the build, e.g. for Makefile which rely on these variables
if [ -z "${CC}" ]; then unset CC; fi
if [ -z "${CFLAGS}" ]; then unset CFLAGS; fi
if [ -z "${CPPFLAGS}" ]; then unset CPPFLAGS; fi
if [ -z "${CXX}" ]; then unset CXX; fi
if [ -z "${CXXFLAGS}" ]; then unset CXXLAGS; fi

if [ -n "$USE_MOCKUP" ]; then
  if [ -z "$TARGET_WORKSPACE" ]; then
    TARGET_WORKSPACE="$USE_MOCKUP"
    ici_warn "Replacing 'USE_MOCKUP=$USE_MOCKUP' with 'TARGET_WORKSPACE=$TARGET_WORKSPACE'"
  else
    ici_error "USE_MOCKUP is not supported anymore, please migrate to 'TARGET_WORKSPACE=$TARGET_WORKSPACE $USE_MOCKUP'"
  fi
fi

TARGET_WORKSPACE=${TARGET_WORKSPACE:-$TARGET_REPO_PATH}

export OS_CODE_NAME
export OS_NAME
export DOCKER_BASE_IMAGE
export ROS_DISTRO
export ROS_VERSION_EOL

# exit with error if OS_NAME is set, but OS_CODE_NAME is not.
# assume ubuntu as default
if [ -z "$OS_NAME" ]; then
    OS_NAME=ubuntu
elif [ -z "$OS_CODE_NAME" ]; then
    ici_error "please specify OS_CODE_NAME"
fi

if [ -n "$UBUNTU_OS_CODE_NAME" ]; then # for backward-compatibility
    OS_CODE_NAME=$UBUNTU_OS_CODE_NAME
fi

if [ -z "$OS_CODE_NAME" ]; then
    case "$ROS_DISTRO" in
    "indigo"|"jade")
        OS_CODE_NAME="trusty"
        ROS_VERSION_EOL=true
        ;;
    "lunar")
        OS_CODE_NAME="xenial"
        ROS_VERSION_EOL=true
        ;;
    "kinetic")
        OS_CODE_NAME="xenial"
        ;;
    "melodic")
        OS_CODE_NAME="bionic"
        ;;
    "")
        if [ -n "$DOCKER_IMAGE" ] || [ -n "$DOCKER_BASE_IMAGE" ]; then
          # try to reed ROS_DISTRO from imgae
          if [ "$DOCKER_PULL" != false ]; then
            docker pull "${DOCKER_IMAGE:-$DOCKER_BASE_IMAGE}"
          fi
          ROS_DISTRO=$(docker image inspect --format "{{.Config.Env}}" "${DOCKER_IMAGE:-$DOCKER_BASE_IMAGE}" | grep -o -P "(?<=ROS_DISTRO=)[a-z]*") || true
        fi
        if [ -z "$ROS_DISTRO" ]; then
            ici_error "Please specify ROS_DISTRO"
        fi
        ;;
    *)
        ici_error "ROS distro '$ROS_DISTRO' is not supported"
        ;;
    esac
fi


function use_snapshot() {
    ROS_REPOSITORY_PATH="http://snapshots.ros.org/${ROS_DISTRO}/$1/ubuntu"
    HASHKEY_SKS="AD19BAB3CBF125EA"
}

function use_repo_or_final_snapshot() {
    if [ "$ROS_VERSION_EOL" = true ]; then
        use_snapshot final
        if [ -n "$ROS_REPO" ]; then
            ici_warn "'$ROS_DISTRO' is in end-of-life state, ROS_REPO='$ROS_REPO' gets ignored"
        fi
    else
        ROS_REPOSITORY_PATH="$1"
        if [ "$ROS_REPO" = "ros-shadow-fixed" ]; then
            ici_warn "ROS_REPO='ros-shadow-fixed' was renamed to ROS_REPO='testing'"
        fi
    fi
}

# If not specified, use ROS Shadow repository http://wiki.ros.org/ShadowRepository
if [ ! "$ROS_REPOSITORY_PATH" ]; then
    case "${ROS_REPO:-testing}" in
    "building")
        use_repo_or_final_snapshot "http://repositories.ros.org/ubuntu/building/"
        ;;
    "ros"|"main")
        use_repo_or_final_snapshot "http://packages.ros.org/ros/ubuntu"
        ;;
    "ros-shadow-fixed"|"ros-testing"|"testing")
        use_repo_or_final_snapshot "http://packages.ros.org/ros-testing/ubuntu"
        ;;
    "final"|????-??-??)
        use_snapshot "${ROS_REPO}"
        ;;
    *)
        ici_error "ROS repo '$ROS_REPO' is not supported"
        ;;
    esac
fi

if [ -z "$DOCKER_BASE_IMAGE" ]; then
    DOCKER_BASE_IMAGE="$OS_NAME:$OS_CODE_NAME" # scheme works for all supported OS images
fi


export TERM=${TERM:-dumb}

# legacy support for UPSTREAM_WORKSPACE and USE_DEB
if [ "$UPSTREAM_WORKSPACE" = "debian" ]; then
  ici_warn "Setting 'UPSTREAM_WORKSPACE=debian' is superfluous and gets removed"
  unset UPSTREAM_WORKSPACE
fi

if [ "$USE_DEB" = true ]; then
  if [ "${UPSTREAM_WORKSPACE:-debian}" != "debian" ]; then
    error "USE_DEB and UPSTREAM_WORKSPACE are in conflict"
  fi
  ici_warn "Setting 'USE_DEB=true' is superfluous"
fi

if [ "$UPSTREAM_WORKSPACE" = "file" ] || [ "${USE_DEB:-true}" != true ]; then
  ROSINSTALL_FILENAME="${ROSINSTALL_FILENAME:-.travis.rosinstall}"
  if [ -f  "$TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO" ]; then
    ROSINSTALL_FILENAME="$ROSINSTALL_FILENAME.$ROS_DISTRO"
  fi

  if [ "${USE_DEB:-true}" != true ]; then # means UPSTREAM_WORKSPACE=file
      if [ "${UPSTREAM_WORKSPACE:-file}" != "file" ]; then
        error "USE_DEB and UPSTREAM_WORKSPACE are in conflict"
      fi
      ici_warn "Replacing 'USE_DEB=false' with 'UPSTREAM_WORKSPACE=$ROSINSTALL_FILENAME'"
  else
      ici_warn "Replacing 'UPSTREAM_WORKSPACE=file' with 'UPSTREAM_WORKSPACE=$ROSINSTALL_FILENAME'"
  fi
  UPSTREAM_WORKSPACE="$ROSINSTALL_FILENAME"
fi
