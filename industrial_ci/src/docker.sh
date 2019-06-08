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

# docker.sh script sets up Docker image.
# It is dependent on environment variables that need to be exported in advance
# (As of version 0.4.4 most of them are defined in ./env.sh).

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

    local docker_target_repo_path=/root/src/$TARGET_REPO_NAME
    local docker_ici_src_path=/root/ici
    ici_run_cmd_in_docker -e "TARGET_REPO_PATH=$docker_target_repo_path" \
                          -v "$TARGET_REPO_PATH/:$docker_target_repo_path:ro" \
                          -v "$ICI_SRC_PATH/:$docker_ici_src_path:ro" \
                          -t \
                          "$DOCKER_IMAGE" \
                          /bin/bash $docker_ici_src_path/ci_main.sh
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
  local run_opts=($DOCKER_RUN_OPTS)
  local commit_image=$DOCKER_COMMIT
  unset DOCKER_COMMIT

  #forward ssh agent into docker container
  if [ "$SSH_AUTH_SOCK" ]; then
     local auth_dir
     auth_dir=$(dirname "$SSH_AUTH_SOCK")
     run_opts+=(-v "$auth_dir:$auth_dir" -e "SSH_AUTH_SOCK=$SSH_AUTH_SOCK")
  fi

  if [ "$CCACHE_DIR" ]; then
     run_opts+=(-v "$CCACHE_DIR:/root/.ccache" -e "CCACHE_DIR=/root/.ccache")
  fi

  if [ -n "$INJECT_QEMU" ]; then
    local qemu_path
    qemu_path=$(which "qemu-$INJECT_QEMU-static") || ici_error "please install qemu-user-static"
    run_opts+=(-v "$qemu_path:$qemu_path:ro")
  fi

  local cid
  cid=$(docker create \
      --env-file "${ICI_SRC_PATH}"/docker.env \
      "${run_opts[@]}" \
      "$@")

  # detect user inside container
  local docker_image
  docker_image=$(docker inspect --format='{{.Config.Image}}' "$cid")
  docker_uid=$(docker run --rm "${run_opts[@]}" "$docker_image" id -u)
  docker_gid=$(docker run --rm "${run_opts[@]}" "$docker_image" id -g)

  # pass common credentials to container
  for d in .docker .ssh .subversion; do
    if [ -d "$HOME/$d" ]; then
      docker_cp "$HOME/$d" "$cid:/root/"
    fi
  done

  docker start -a "$cid" &
  trap 'docker kill $cid' INT
  local ret=0
  wait %% || ret=$?
  trap - INT
  if [ -n "$commit_image" ]; then
    echo "Committing container to tag: '$commit_image'"
    ici_quiet docker commit -m "$DOCKER_COMMIT_MSG" "$cid" "$commit_image"
  fi
  ici_quiet docker rm "$cid"
  return $ret
}

# work-around for https://github.com/moby/moby/issues/34096
# ensures that copied files are owned by the target user
function docker_cp {
  set -o pipefail
  tar --numeric-owner --owner="${docker_uid:-root}" --group="${docker_gid:-root}" -c -f - -C "$(dirname "$1")" "$(basename "$1")" | docker cp - "$2"
  set +o pipefail
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
  if [ "$DOCKER_PULL" != false ]; then
    opts+=("--pull")
  fi
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
       ici_quiet ici_docker_build - < "$TARGET_REPO_PATH/$DOCKER_FILE"
    elif [ -d "$TARGET_REPO_PATH/$DOCKER_FILE" ]; then # if path, run with context
        ici_quiet ici_docker_build "$TARGET_REPO_PATH/$DOCKER_FILE"
    else # url, run directly
        ici_quiet ici_docker_build "$DOCKER_FILE"
    fi
  elif [ -z "$DOCKER_IMAGE" ]; then # image was not provided, use default
     ici_build_default_docker_image
  elif [ "$DOCKER_PULL" != false ]; then
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
  if [ -n "$INJECT_QEMU" ]; then
    local qemu_path
    qemu_path=$(which "qemu-$INJECT_QEMU-static") || ici_error "please install qemu-user-static"
    echo "Inject qemu..."
    local qemu_temp
    qemu_temp=$(mktemp -d)
    cat <<EOF > "$qemu_temp/Dockerfile"
    FROM $DOCKER_BASE_IMAGE
    COPY '$(basename "$qemu_path")' '$qemu_path'
EOF
    cp "$qemu_path" "$qemu_temp"
    unset INJECT_QEMU
    export DOCKER_BASE_IMAGE="$DOCKER_BASE_IMAGE-qemu"
    DOCKER_IMAGE="$DOCKER_BASE_IMAGE" ici_quiet ici_docker_build "$qemu_temp"
    rm -rf "$qemu_temp"
  fi
  # choose a unique image name
  export DOCKER_IMAGE="industrial-ci/$ROS_DISTRO/$DOCKER_BASE_IMAGE"
  echo "Building image '$DOCKER_IMAGE':"
  local dockerfile; dockerfile=$(ici_generate_default_dockerfile)
  echo "$dockerfile"
  ici_quiet ici_docker_build - <<< "$dockerfile"
}

function ici_generate_default_dockerfile() {
  local keycmd

  if [ -n "${APTKEY_STORE_HTTPS}" ]; then
    keycmd="wget '${APTKEY_STORE_HTTPS}' -O - | apt-key add -"
  else
    keycmd="apt-key adv --keyserver '${APTKEY_STORE_SKS}' --recv-key '${HASHKEY_SKS}'"
  fi

  cat <<EOF
FROM $DOCKER_BASE_IMAGE

ENV ROS_DISTRO $ROS_DISTRO

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y apt-utils gnupg wget ca-certificates lsb-release

RUN echo "deb ${ROS_REPOSITORY_PATH} \$(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN for i in 1 2 3; do { $keycmd; } &&  break || sleep 1; done

RUN sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list \
    && apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y \
        build-essential \
        python-catkin-tools \
        python-pip \
        python-rosdep \
        python-wstool \
        ros-$ROS_DISTRO-catkin \
        ssh-client \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
EOF
}
