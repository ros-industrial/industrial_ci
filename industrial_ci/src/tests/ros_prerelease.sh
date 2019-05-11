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

function setup_ros_prerelease() {
    useradd -m -d "$WORKSPACE/home" ci

    if [ -e /var/run/docker.sock ]; then
        groupadd -o -g "$(stat -c%g /var/run/docker.sock)" host_docker
        usermod -a -G host_docker ci
    fi

    ici_quiet ici_asroot apt-get -qq install -y docker.io python3-pip python3-yaml sudo
    ici_asroot pip3 install git+https://github.com/ros-infrastructure/ros_buildfarm.git
}

function prepare_prerelease_workspaces() {
  local -a ws_upstream ws_target
  ici_parse_env_array ws_upstream UPSTREAM_WORKSPACE
  ici_parse_env_array ws_target TARGET_WORKSPACE
  local workspace=$1
  local reponame=$2
  local targetname=$3
  ici_with_ws "$workspace/ws" ici_prepare_sourcespace "$workspace/ws/src/" "${ws_upstream[@]}" "${ws_target[@]}"

  if ! [ -d "$workspace/ws/src/$reponame" ]; then
      mv "$workspace/ws/src/$targetname" "$workspace/ws/src/$reponame"
  fi

  local -a overlay
  ici_parse_env_array overlay DOWNSTREAM_WORKSPACE
  ici_with_ws "$workspace/ws_overlay" ici_prepare_sourcespace "$workspace/ws_overlay/src/" "${overlay[@]}"
  chown -R ci "$workspace"
}

function run_ros_prerelease() {
    local ws; ws=$(mktemp -d)
    export WORKSPACE
    local -a opts
    ici_parse_env_array opts DOCKER_RUN_OPTS
    opts+=(-e TRAVIS -e OS_NAME -e OS_CODE_NAME -e OS_ARCH -e PRERELEASE_DOWNSTREAM_DEPTH -e PRERELEASE_REPONAME -e ROSDISTRO_INDEX_URL
                 -v "$ws:$ws:rw" -e "WORKSPACE=$ws")

    if [ -n "$DOCKER_PORT" ]; then
        opts+=(-e "DOCKER_HOST=$DOCKER_PORT")
    elif [ -e /var/run/docker.sock ]; then
        opts+=(-v /var/run/docker.sock:/var/run/docker.sock)
    fi
    DOCKER_RUN_OPTS="${opts[*]}" DOCKER_IMAGE=${DOCKER_IMAGE:-ros:$ROS_DISTRO-ros-core} ici_require_run_in_docker

    ici_run "setup_ros_prerelease" setup_ros_prerelease

    # Environment vars.
    local downstream_depth=${PRERELEASE_DOWNSTREAM_DEPTH:-"0"}
    local reponame=${PRERELEASE_REPONAME:-$TARGET_REPO_NAME}

    if [ -z "$ROSDISTRO_INDEX_URL" ]; then
      if [ "$ROS_VERSION" -eq 2 ]; then
          ROSDISTRO_INDEX_URL="https://raw.githubusercontent.com/ros2/ros_buildfarm_config/ros2/index.yaml"
          ici_quiet ici_install_pkgs_for_command colcon python3-colcon-common-extensions
      else
          ROSDISTRO_INDEX_URL="https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml"
      fi
    fi

    ici_run "prepare_prerelease_workspaces" prepare_prerelease_workspaces "$WORKSPACE" "$reponame" "$(basename "$TARGET_REPO_PATH")"
    ici_run 'generate_prerelease_script' sudo -EH -u ci generate_prerelease_script.py "${ROSDISTRO_INDEX_URL}" "$ROS_DISTRO" default "$OS_NAME" "$OS_CODE_NAME" "${OS_ARCH:-amd64}" --level "$downstream_depth" --output-dir "$WORKSPACE" --custom-repo "$reponame::::"
    ABORT_ON_TEST_FAILURE=1 ici_run "run_prerelease_script" sudo -EH -u ci sh -c "mkdir -p ~/.ccache && . '/opt/ros/$ROS_DISTRO/setup.sh' && cd '$WORKSPACE' && exec ./prerelease.sh -y"

    echo 'ROS Prerelease Test went successful.'
}
