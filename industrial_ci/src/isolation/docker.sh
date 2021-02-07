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
export DOCKER_CREDENTIALS=${DOCKER_CREDENTIALS-.docker .ssh .subversion}
export DOCKER_PULL=${DOCKER_PULL:-true}

# ici_forward_mount VARNAME/FILE rw/ro [PATH]
function ici_forward_mount() {
    local p=$1
    local v=
    if ! [ -e "$1" ]; then
        v=$1
        p=${!1:-}
    fi
    if [ -n "$p" ]; then
        local p_abs
        p_abs=$(readlink -m "$p")
        local p_inner=${3:-$p_abs}
        _docker_run_opts+=(-v "$p_abs:$p_inner:$2")
        if [ -n "$v" ]; then
            ici_forward_variable "$v" "$p_inner"
        fi
    fi
}

# ici_forward_variable VARNAME [VALUE]
function ici_forward_variable() {
    if [ -n "${2-}" ]; then
        _docker_run_opts+=(-e "$1=$2")
    else
        _docker_run_opts+=(-e "$1")
    fi
}

#######################################
# rerun the CI script in docker container end exit the outer script
#
# Globals:
#   DOCKER_IMAGE (read-only)
#   ICI_SRC_PATH (read-only)
#   TARGET_REPO_PATH (read-only)
# Arguments:
#   (None)
# Returns:
#   (None)
#######################################
function ici_isolate() {
    local file=${1}
    shift

    if [ "${DOCKER_IMAGE-x}" = "" ]; then
        ici_error "Empty string passed to DOCKER_IMAGE. Specify a valid docker image or unset the environment variable to use the default image."
    fi

    if [ -n "${OS_CODE_NAME-}" ]; then
        DOCKER_IMAGE=${DOCKER_IMAGE:-${OS_NAME}:$OS_CODE_NAME} # scheme works for all supported OS images
    elif [ -z "${DOCKER_IMAGE-}" ]; then
        ici_error "Please set ROS_DISTRO, OS_CODE_NAME or DOCKER_IMAGE."
    fi

    if [ "$DOCKER_PULL" != false ]; then
        ici_run "pull_docker_image" docker pull "$DOCKER_IMAGE"
    fi

    if [ -z "${ROS_DISTRO:-}" ]; then
        ROS_DISTRO=$(docker image inspect --format "{{.Config.Env}}" "${DOCKER_IMAGE}" | grep -o -P "(?<=ROS_DISTRO=)[a-z]*") || ici_error "ROS_DISTRO is not set"
    elif [ "${ROS_DISTRO}" = "false" ]; then
        unset ROS_DISTRO
    fi

    if [ -n "${BASEDIR:-}" ]; then
        # $BASEDIR is most-likely contained in $TARGET_REPO_PATH
        # copy target repo to temporary folder first
        local tmp_src
        ici_make_temp_dir tmp_src
        cp -a "$TARGET_REPO_PATH" "$tmp_src/"
        export TARGET_REPO_PATH
        TARGET_REPO_PATH="$tmp_src/$(basename "$TARGET_REPO_PATH")"
    fi

    ici_forward_mount TARGET_REPO_PATH ro
    ici_forward_mount ICI_SRC_PATH ro
    ici_forward_mount BASEDIR rw
    ici_forward_mount CCACHE_DIR rw
    ici_forward_mount SSH_AUTH_SOCK rw # forward ssh agent into docker container

    local run_opts
    ici_parse_env_array run_opts DOCKER_RUN_OPTS

    for hook in $(env | grep -o '^\(BEFORE\|AFTER\)_[^=]*'); do
        ici_forward_variable "$hook"
    done

    ici_run_cmd_in_docker "${_docker_run_opts[@]}" "${run_opts[@]}" \
        -t \
        --entrypoint '' \
        -w "$TARGET_REPO_PATH" \
        "$DOCKER_IMAGE" \
        /bin/bash "$ICI_SRC_PATH/run.sh" "$file" "$@"
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
    local credentials=()
    ici_parse_env_array credentials DOCKER_CREDENTIALS
    local to_copy=()
    local cleanup=""

    for d in "${credentials[@]}"; do
        if [ -d "$HOME/$d" ]; then
            to_copy+=(~/"$d")
            # shellcheck disable=SC2088
            cleanup=$(ici_join_array : "$cleanup" "~/$d")
        fi
    done

    local opts=(--env-file "${ICI_SRC_PATH}/isolation/docker.env")
    if [ -z "$DOCKER_COMMIT" ]; then
        opts+=(--rm)
    else
        opts+=(-e "_CLEANUP=$cleanup")
    fi

    local cid
    cid=$(docker create "${opts[@]}" "$@")

    # detect user inside container
    local image
    image=$(docker inspect --format='{{.Config.Image}}' "$cid")
    local docker_query=()
    # shellcheck disable=SC2016
    IFS=" " read -r -a docker_query <<<"$(docker run --rm --entrypoint '/bin/sh' "$image" -c 'echo "$(id -u) $(id -g) $HOME"')"

    # pass common credentials to container
    for d in "${to_copy[@]}"; do
        ici_warn "Copy credentials: $d"
        docker_cp "$d" "$cid:${docker_query[*]:2}/" "${docker_query[0]}" "${docker_query[1]}"
    done

    docker start -a "$cid" &
    trap 'docker kill --signal=SIGTERM $cid' INT
    local ret=0
    wait %% || ret=$?
    trap - INT
    if [ -n "$DOCKER_COMMIT" ]; then
        echo "Committing container to tag: '$DOCKER_COMMIT'"
        local msg=()
        if [ -n "$DOCKER_COMMIT_MSG" ]; then
            msg=(-m "$DOCKER_COMMIT_MSG")
        fi
        ici_quiet docker commit "${msg[@]}" "$cid" "$DOCKER_COMMIT"
        ici_quiet docker rm "$cid"
    fi
    return $ret
}

# work-around for https://github.com/moby/moby/issues/34096
# ensures that copied files are owned by the target user
function docker_cp() {
    set -o pipefail
    tar --numeric-owner --owner="${3:-root}" --group="${4:-root}" -c -f - -C "$(dirname "$1")" "$(basename "$1")" | docker cp - "$2"
    set +o pipefail
}
