#!/bin/bash

# Copyright (c) 2016, Isaac I. Y. Saito
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

# ros_prerelease.sh script runs ROS Pre-release Test.
# It is dependent on environment variables that need to be exported in advance
# (As of version 0.4.4 most of them are defined in env.sh).

function setup_ros_buildfarm() {
    ici_install_pkgs_for_command pip3 python3-pip python3-setuptools python3-wheel
    ici_cmd ici_quiet ici_asroot pip3 install git+https://github.com/ros-infrastructure/ros_buildfarm.git
}

function setup_ros_prerelease() {
    ici_asroot useradd -m -d "$WORKSPACE/home" ci

    if ! [ -d "$WORKSPACE/home/.ccache" ]; then
      ici_asroot mkdir -p "$WORKSPACE/home/.ccache"
    fi

    if [ -e /var/run/docker.sock ]; then
        ici_asroot groupadd -o -g "$(stat -c%g /var/run/docker.sock)" host_docker
        ici_asroot usermod -a -G host_docker ci
    fi

    ici_setup_git_client
    ici_install_pkgs_for_command docker docker.io
    ici_install_pkgs_for_command sudo sudo
    ici_install_pkgs_for_command lsb_release lsb-release
    ici_exec_for_command generate_prerelease_script.py setup_ros_buildfarm
}

function prepare_prerelease_workspaces() {
  local ws_upstream=()
  ici_parse_env_array ws_upstream UPSTREAM_WORKSPACE
  local ws_target=()
  ici_parse_env_array ws_target TARGET_WORKSPACE
  local workspace=$1
  local reponame=$2
  local targetname=$3
  ici_with_ws "$workspace/ws" ici_prepare_sourcespace "$workspace/ws/src/" "${ws_upstream[@]}" "${ws_target[@]}"

  if ! [ -d "$workspace/ws/src/$reponame" ]; then
      mv "$workspace/ws/src/$targetname" "$workspace/ws/src/$reponame"
  fi

  local overlay=()
  ici_parse_env_array overlay DOWNSTREAM_WORKSPACE
  ici_with_ws "$workspace/ws_overlay" ici_prepare_sourcespace "$workspace/ws_overlay/src/" "${overlay[@]}"
  ici_asroot chown -R ci "$workspace"
}

function prepare_ros_prerelease() {
    if [ "$BUILDER" != "colcon" ]; then
        export BUILDER=catkin_make_isolated
    fi
    export WORKSPACE; WORKSPACE=$(mktemp -d)
    if [ -z "${ROSDISTRO_INDEX_URL:-}" ]; then
      if [ "$ROS_VERSION" -eq 2 ]; then
          export ROSDISTRO_INDEX_URL="https://raw.githubusercontent.com/ros2/ros_buildfarm_config/ros2/index.yaml"
      else
          export ROSDISTRO_INDEX_URL="https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml"
      fi
    fi
    export PRERELEASE_DISTRO="$ROS_DISTRO"

    ici_parse_env_array opts DOCKER_RUN_OPTS
    for e in TRAVIS OS_NAME OS_CODE_NAME OS_ARCH PRERELEASE_DOWNSTREAM_DEPTH PRERELEASE_REPONAME ROSDISTRO_INDEX_URL PRERELEASE_DISTRO; do
        ici_forward_variable "$e"
    done

    ici_forward_mount WORKSPACE rw

    if [ -n "${DOCKER_PORT:-}" ]; then
        ici_forward_variable DOCKER_HOST "$DOCKER_PORT"
    elif [ -e /var/run/docker.sock ]; then
        ici_forward_mount /var/run/docker.sock rw
    fi
    if [ -n "${CCACHE_DIR}" ]; then
      ici_forward_mount CCACHE_DIR rw "$WORKSPACE/home/.ccache"
      CCACHE_DIR= # prevent cachedir from beeing added twice
    fi
    export DOCKER_IMAGE=${DOCKER_IMAGE:-ros:noetic-ros-core}
    export ROS_DISTRO=noetic
}

function run_ros_prerelease() {
    ici_source_builder
    ici_step "${BUILDER}_setup" builder_setup

    ici_step "setup_ros_prerelease" setup_ros_prerelease

    # Environment vars.
    local downstream_depth=${PRERELEASE_DOWNSTREAM_DEPTH:-"0"}
    local reponame=${PRERELEASE_REPONAME:-$TARGET_REPO_NAME}

    ici_step "prepare_prerelease_workspaces" ici_cmd prepare_prerelease_workspaces "$WORKSPACE" "$reponame" "$(basename "$TARGET_REPO_PATH")"
    ici_step 'generate_prerelease_script' ici_cmd sudo -EH -u ci generate_prerelease_script.py "${ROSDISTRO_INDEX_URL}" "$PRERELEASE_DISTRO" default "$OS_NAME" "$OS_CODE_NAME" "${OS_ARCH:-amd64}" --build-tool "$BUILDER" --level "$downstream_depth" --output-dir "$WORKSPACE" --custom-repo "$reponame::::"

    local setup_sh=
    if [ -f "${UNDERLAY:?}/setup.sh" ]; then
        setup_sh=". $UNDERLAY/setup.sh && "
    fi
    ABORT_ON_TEST_FAILURE=1 ici_step "run_prerelease_script" ici_cmd sudo -EH -u ci sh -c "${setup_sh}cd '$WORKSPACE' && exec ./prerelease.sh -y"

    ici_log 'ROS Prerelease Test went successful.'
}
