#!/bin/bash

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

#######################################
# rerun the CI script in docker container end exit the outer script
#
# Globals:
#   DOCKER_IMAGE (read-only)
#   ICI_SRC_PATH (read-only)
#   IN_DOCKER (read-only)
#   TARGET_REPO_PATH (read-only)
# Arguments:
#   (None)
# Returns:
#   (None)
#######################################
function ici_require_run_in_docker() {
  if ! [ "$IN_DOCKER" ]; then
    ici_prepare_docker_image

    local docker_target_repo_path=/root/$TARGET_REPO_PATH
    local docker_ici_src_path=/root/ici
    ici_run_cmd_in_docker -e "TARGET_REPO_PATH=$docker_target_repo_path" \
                          -v "$TARGET_REPO_PATH/:$docker_target_repo_path:ro" \
                          -v "$ICI_SRC_PATH/:$docker_ici_src_path:ro" \
                          "$DOCKER_IMAGE" \
                          /bin/bash -c "cd $docker_ici_src_path; source ./ci_main.sh;"
    exit
  fi
}

#######################################
# wrapper for running a command in docker
#
# * enables environment passing
# * set-ups SSH auth socket forwarding
# * stops on interrupt signal
#
# Globals:
#   ICI_SRC_PATH (read-only)
#   SSH_AUTH_SOCK (read-only)
# Arguments:
#   all argumentes will be forwarded
# Returns:
#   (None)
#######################################
function ici_run_cmd_in_docker() {
  #forward ssh agent into docker container
 local ssh_docker_opts=()
  if [ "$SSH_AUTH_SOCK" ]; then
     local auth_dir
     auth_dir=$(dirname "$SSH_AUTH_SOCK")
     ssh_docker_opts=(-v "$auth_dir:$auth_dir" -e "SSH_AUTH_SOCK=$SSH_AUTH_SOCK")
  fi

  local cid
  cid=$(docker create \
      --env-file "${ICI_SRC_PATH}"/docker.env \
      "${ssh_docker_opts[@]}" \
      "$@")
  docker cp ~/.ssh "$cid:/root/" # pass SSH settings to container

  docker start -a "$cid" &
  trap 'docker kill $cid' INT
  local ret=0
  wait %% || ret=$?
  trap - INT
  docker rm "$cid"
  return $ret
}

#######################################
# wrapper for docker build
#
# * images will by tagged automatically
# * build option get passed from environment
#
# Globals:
#   DOCKER_BUILD_OPTS (read-only)
#   DOCKER_IMAGE (read-only)
# Arguments:
#   all argumentes will be forwarded
# Returns:
#   (None)
#######################################
function ici_docker_build() {
  local opts=($DOCKER_BUILD_OPTS)
  docker build -t "$DOCKER_IMAGE" "${opts[@]}" "$@"
}

#######################################
# set-ups the CI docker image
#
# * pull or build custom image
# * fall-bak to default build
#
# Globals:
#   DOCKER_FILE (read-only)
#   DOCKER_IMAGE (read/write)
#   TARGET_REPO_PATH (read-only)
# Arguments:
#   (None)
# Returns:
#   (None)
function ici_prepare_docker_image() {
  ici_time_start prepare_docker_image

  if [ -n "$DOCKER_FILE" ]; then # docker file was provided
    DOCKER_IMAGE=${DOCKER_IMAGE:"industrial-ci/custom"}
    if [ -f "$TARGET_REPO_PATH/$DOCKER_FILE" ]; then # if single file, run without context
       ici_docker_build - < "$TARGET_REPO_PATH/$DOCKER_FILE" > /dev/null
    elif [ -d "$TARGET_REPO_PATH/$DOCKER_FILE" ]; then # if path, run with context
        ici_docker_build "$TARGET_REPO_PATH/$DOCKER_FILE" > /dev/null
    else # url, run directly
        ici_docker_build "$DOCKER_FILE" > /dev/null
    fi
  elif [ -z "$DOCKER_IMAGE" ]; then # image was not provided, use default
     ici_build_default_docker_image
  else
     docker pull "$DOCKER_IMAGE"
  fi
  ici_time_end # prepare_docker_image
}

#######################################
# build the default docker image
#
# Globals:
#   APTKEY_STORE_HTTPS (read-only)
#   APTKEY_STORE_SKS (read-only)
#   DOCKER_IMAGE (write-only)
#   HASHKEY_SKS (read-only)
#   UBUNTU_OS_CODE_NAME (read-only)
# Arguments:
#   (None)
# Returns:
#   (None)

function ici_build_default_docker_image() {
  local os_code_name=$UBUNTU_CODE_NAME
  if [ -z "$os_code_name" ]; then
    case "$ROS_DISTRO" in
    "hydro")
        os_code_name="precise"
        ;;
    "kinetic")
        os_code_name="xenial"
        ;;
    *)
        os_code_name=$(lsb_release -sc)
        ;;
    esac
  fi
  export DOCKER_IMAGE="industrial-ci/$os_code_name"
  
  ici_docker_build - <<EOF > /dev/null
FROM ubuntu:$os_code_name

RUN apt-get update -qq && apt-get -qq install --no-install-recommends -y wget ca-certificates

RUN echo "deb ${ROS_REPOSITORY_PATH} $os_code_name main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver "${APTKEY_STORE_SKS}" --recv-key "${HASHKEY_SKS}" \
    || { wget "${APTKEY_STORE_HTTPS}" -O - | sudo apt-key add -; }

RUN apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y \
        build-essential \
        python-catkin-tools \
        python-pip \
        python-rosdep \
        python-wstool \
        ros-$ROS_DISTRO-catkin \
        ssh-client \
        sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
EOF
}
