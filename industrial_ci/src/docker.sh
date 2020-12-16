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
  if [ "$_IN_DOCKER" != true ]; then
    # fallback to default base image
    if [ -z "$DOCKER_IMAGE" ]; then
      DOCKER_IMAGE="$OS_NAME:$OS_CODE_NAME" # scheme works for all supported OS images
    fi

    ici_run "prepare_docker_image" ici_docker_try_pull "$DOCKER_IMAGE"

    local docker_target_repo_path=/root/src/$TARGET_REPO_NAME
    local docker_ici_src_path=/root/ici
    local testpath="${BASH_SOURCE[1]/#$ICI_SRC_PATH/$docker_ici_src_path}"
    local testpath="${testpath/#$ICI_SRC_PATH/$docker_ici_src_path}"
    ici_run_cmd_in_docker -e "TARGET_REPO_PATH=$docker_target_repo_path" \
                          -v "$TARGET_REPO_PATH/:$docker_target_repo_path:ro" \
                          -e "ICI_SRC_PATH=$docker_ici_src_path" \
                          -v "$ICI_SRC_PATH/:$docker_ici_src_path:ro" \
                          -t \
                          --entrypoint '' \
                          -w "$docker_target_repo_path" \
                          "$DOCKER_IMAGE" \
                          /bin/bash $docker_ici_src_path/run.sh "${BASH_SOURCE[1]/#$ICI_SRC_PATH/$docker_ici_src_path}"
    exit
  else
    export LANG=${LANG:-C.UTF-8}
    export LC_ALL=${LC_ALL:-C.UTF-8}
    export TERM=${TERM:-dumb}

    if [ -z "${CC}" ]; then unset CC; fi
    if [ -z "${CFLAGS}" ]; then unset CFLAGS; fi
    if [ -z "${CPPFLAGS}" ]; then unset CPPFLAGS; fi
    if [ -z "${CXX}" ]; then unset CXX; fi
    if [ -z "${CXXFLAGS}" ]; then unset CXXLAGS; fi

    ici_run "init" ici_init_apt
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
  local -a run_opts
  ici_parse_env_array run_opts DOCKER_RUN_OPTS
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

  local hooks=()
  for hook in $(env | grep -o '^\(BEFORE\|AFTER\)_[^=]*'); do
      hooks+=(-e "$hook")
  done
  local cid
  cid=$(docker create \
      --env-file "${ICI_SRC_PATH}"/docker.env \
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
function ici_docker_try_pull {
    local image=$1
    if [ -z "$image" ]; then
      ici_error "Could not determine Docker image"
    fi
    if [ "$DOCKER_PULL" != false ]; then
        echo "Pulling Docker image '$image'..."
        ici_quiet docker pull "$image"
    fi
}
