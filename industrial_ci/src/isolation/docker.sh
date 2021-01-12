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

export DOCKER_COMMIT=${DOCKER_COMMIT:-}
export DOCKER_COMMIT_MSG=${DOCKER_COMMIT_MSG:-}
export DOCKER_COMMIT_CREDENTIALS=${DOCKER_COMMIT_CREDENTIALS:-false}
export DOCKER_PULL=${DOCKER_PULL:-true}

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
function ici_isolate() {
  local file=${1}; shift
  DOCKER_IMAGE=${DOCKER_IMAGE:-${OS_NAME:-ubuntu}:$OS_CODE_NAME} # scheme works for all supported OS images

  if [ "$DOCKER_PULL" != false ]; then
      ici_run "pull_docker_image"  docker pull "$DOCKER_IMAGE"
  fi

  if [ -z "${ROS_DISTRO:-}" ]; then
      ROS_DISTRO=$(docker image inspect --format "{{.Config.Env}}" "${DOCKER_IMAGE}" | grep -o -P "(?<=ROS_DISTRO=)[a-z]*") || ici_error "ROS_DISTRO is not set"
  elif [ "${ROS_DISTRO}" = "false" ]; then
      unset ROS_DISTRO
  fi

  local docker_target_repo_path=/root/src/$TARGET_REPO_NAME
  local docker_ici_src_path=/root/ici
  file="${file/#$TARGET_REPO_PATH/$docker_target_repo_path}"
  file="${file/#$ICI_SRC_PATH/$docker_ici_src_path}"

  ici_run_cmd_in_docker -e "TARGET_REPO_PATH=$docker_target_repo_path" \
                        -v "$TARGET_REPO_PATH/:$docker_target_repo_path:ro" \
                        -e "ICI_SRC_PATH=$docker_ici_src_path" \
                        -v "$ICI_SRC_PATH/:$docker_ici_src_path:ro" \
                        -t \
                        --entrypoint '' \
                        -w "$docker_target_repo_path" \
                        "$DOCKER_IMAGE" \
                        /bin/bash $docker_ici_src_path/run.sh "$file" "$@"
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
  local run_opts=()
  ici_parse_env_array run_opts DOCKER_RUN_OPTS
  local commit_image=$DOCKER_COMMIT
  DOCKER_COMMIT=

  #forward ssh agent into docker container
  if [ -n "${SSH_AUTH_SOCK:-}" ]; then
     local auth_dir
     auth_dir=$(dirname "$SSH_AUTH_SOCK")
     run_opts+=(-v "$auth_dir:$auth_dir" -e "SSH_AUTH_SOCK=$SSH_AUTH_SOCK")
  fi

  if [ -n "${BASEDIR-}" ]; then
    mkdir -p "$BASEDIR"
    run_opts+=(-v "$BASEDIR:$BASEDIR" -e "BASEDIR=$BASEDIR")
  fi

  if [ -n "${CCACHE_DIR}" ]; then
     run_opts+=(-v "$CCACHE_DIR:/root/.ccache" -e "CCACHE_DIR=/root/.ccache")
  fi

  local hooks=()
  for hook in $(env | grep -o '^\(BEFORE\|AFTER\)_[^=]*'); do
      hooks+=(-e "$hook")
  done
  local cid
  cid=$(docker create \
      --env-file "${ICI_SRC_PATH}/isolation/docker.env" \
      "${hooks[@]}" \
      "${run_opts[@]}" \
      "$@")

  # detect user inside container
  local image
  image=$(docker inspect --format='{{.Config.Image}}' "$cid")
  docker_uid=$(docker run --rm "${run_opts[@]}" --entrypoint '' "$image" id -u)
  docker_gid=$(docker run --rm "${run_opts[@]}" --entrypoint '' "$image" id -g)

  # pass common credentials to container
  if [ "$DOCKER_COMMIT_CREDENTIALS" != false ]; then
    for d in .docker .ssh .subversion; do
      if [ -d "$HOME/$d" ]; then
        if [ -z "$commit_image" ] || [ "$DOCKER_COMMIT_CREDENTIALS" = true ]; then
          docker_cp "$HOME/$d" "$cid:/root/"
        else
          ici_warn "Will not bundle'$d' unless 'DOCKER_COMMIT_CREDENTIALS=true'"
        fi
      fi
    done
  fi

  docker start -a "$cid" &
  trap 'docker kill $cid' INT
  local ret=0
  wait %% || ret=$?
  trap - INT
  if [ -n "$commit_image" ]; then
    echo "Committing container to tag: '$commit_image'"
    local msg=()
    if [ -n "$DOCKER_COMMIT_MSG" ]; then
      msg=(-m "$DOCKER_COMMIT_MSG")
    fi
    ici_quiet docker commit "${msg[@]}" "$cid" "$commit_image"
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
