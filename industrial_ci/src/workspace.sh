#!/bin/bash

# Copyright (c) 2019, Mathias Lüdtke
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
#

export _DEFAULT_DEBS=${_DEFAULT_DEBS:-}

function ici_resolve_scheme {
    local url=$1; shift
    if [[ $url =~ ([^:]+):([^#@]+)[#@](.+) ]]; then
        local fragment="${BASH_REMATCH[3]}"
        local repo=${BASH_REMATCH[2]}
        local name=${repo##*/}
        local scheme=${BASH_REMATCH[1]}

        case "$scheme" in
            bitbucket | bb)
                echo "${name%.git}" "git" "https://bitbucket.org/$repo" "$fragment"
                ;;
            github | gh)
                echo "${name%.git}" "git" "https://github.com/$repo" "$fragment"
                ;;
            gitlab | gl)
                echo "${name%.git}" "git" "https://gitlab.com/$repo" "$fragment"
                ;;
            'git+file'*|'git+http'*)
                echo "${name%.git}" "git" "${scheme#git+}:$repo" "$fragment"
                ;;
            git+*)
                echo "${name%.git}" "git" "$scheme:$repo" "$fragment"
                ;;
            *)
                echo "$name" "$scheme" "$scheme:$repo" "$fragment"
                ;;
        esac
    else
        ici_error "Could not parse URL '$url'. It does not match the expected pattern: <scheme>:<resource>#<version>."
    fi

}

function ici_apt_install {
    ici_asroot apt-get -qq install -y --no-upgrade --no-install-recommends "$@"
}

function ici_pip_install {
    ici_asroot "${PYTHON_VERSION_NAME}" -m pip install -q "$@"
}

function ici_init_apt {
    export DEBIAN_FRONTEND=noninteractive

    ici_asroot sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list
    ici_asroot apt-get update -qq

    local debs_default=(apt-utils build-essential)
    if ! ls /etc/ssl/certs/* 2&> /dev/null; then
        debs_default+=(ca-certificates)
    fi
    if [ -n "$_DEFAULT_DEBS" ]; then
        ici_parse_env_array debs_default _DEFAULT_DEBS
    fi
    if [ -n "${debs_default[*]}" ]; then
        ici_apt_install "${debs_default[@]}"
    fi

    if ! [ -f "/etc/apt/sources.list.d/ros${ROS_VERSION}-latest.list" ]; then
        ici_install_pkgs_for_command lsb_release lsb-release

        local keycmd
        if [ -n "${APTKEY_STORE_HTTPS}" ]; then
            ici_install_pkgs_for_command wget wget
            keycmd="wget '${APTKEY_STORE_HTTPS}' -O - | ici_asroot apt-key add -"
        else
            keycmd="ici_asroot apt-key adv --keyserver '${APTKEY_STORE_SKS:-hkp://keyserver.ubuntu.com:80}' --recv-key '${HASHKEY_SKS:-C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654}'"
            ici_apt_install gnupg2 dirmngr
        fi

        ici_retry 3 eval "$keycmd"
        ici_asroot tee "/etc/apt/sources.list.d/ros${ROS_VERSION}-latest.list" <<< "deb ${ROS_REPOSITORY_PATH} $(lsb_release -sc) main" > /dev/null
        ici_asroot apt-get update -qq
    fi

    # If more DEBs needed during preparation, define ADDITIONAL_DEBS variable where you list the name of DEB(S, delimitted by whitespace)
    local debs=()
    ici_parse_env_array debs ADDITIONAL_DEBS
    if [ -n "${debs[*]}" ]; then
        ici_apt_install "${debs[@]}" || ici_error "One or more additional deb installation is failed. Exiting."
    fi
}

function ici_install_pkgs_for_command {
  local command=$1; shift
  ici_exec_for_command "$command" ici_apt_install "$@"
}

function ici_install_pypi_pkgs_for_command {
  local command=$1; shift
  ici_exec_for_command "$command" ici_pip_install "$@"
}

function ici_setup_git_client {
  ici_install_pkgs_for_command git git-core
  if [ -d ~/.ssh ]; then
    ici_install_pkgs_for_command ssh ssh-client
  fi
}

function ici_vcs_import {
    vcs import --recursive --force "$@"
}

function ici_import_repository {
    local sourcespace=$1; shift
    local url=$1; shift

    ici_install_pkgs_for_command vcs python3-vcstool

    local resolved; resolved=$(ici_resolve_scheme "$url")
    IFS=" " read -r -a parts <<< "$resolved" # name, type, url, version
    echo "${parts[*]}"

    case "${parts[1]}" in
        git)
          ici_setup_git_client
            ;;
        *)
            ;;
    esac
    if [ "${parts[3]}" = "HEAD" ]; then
        ici_vcs_import "$sourcespace" <<< "{repositories: {'${parts[0]}': {type: '${parts[1]}', url: '${parts[2]}'}}}"
    else
        ici_vcs_import "$sourcespace" <<< "{repositories: {'${parts[0]}': {type: '${parts[1]}', url: '${parts[2]}', version: '${parts[3]}'}}}"
    fi
}

function ici_import_file {
    local sourcespace=$1; shift
    local file=$1; shift

    case "$file" in
    *.zip|*.tar|*.tar.*|*.tgz|*.tbz2)
        ici_install_pkgs_for_command bsdtar bsdtar
        bsdtar -C "$sourcespace" -xf "$file"
        ;;
    *)
        ici_install_pkgs_for_command vcs python3-vcstool
        ici_setup_git_client
        ici_vcs_import "$sourcespace" < "$file"
    ;;
    esac

}

function ici_import_url {
    local sourcespace=$1; shift
    local url=$1; shift
    local processor

    ici_install_pkgs_for_command wget wget

    case "$url" in
    *.zip|*.tar|*.tar.*|*.tgz|*.tbz2)
        ici_install_pkgs_for_command bsdtar bsdtar
        processor=(bsdtar -C "$sourcespace" -xf-)
        ;;
    *)
        ici_install_pkgs_for_command vcs python3-vcstool
        ici_setup_git_client
        processor=(ici_vcs_import "$sourcespace")
    ;;
    esac

    set -o pipefail
    wget -O- -q "$url" | "${processor[@]}"
    set +o pipefail
}

function  ici_import_directory {
    local sourcespace=$1; shift
    local dir=$1; shift
    rm -rf "$sourcespace:?/$(basename "$dir")"
    cp -a "$dir" "$sourcespace"
}

function ici_prepare_sourcespace {
    local sourcespace=$1; shift
    local basepath=$TARGET_REPO_PATH

    mkdir -p "$sourcespace"

    for source in "$@"; do
        case "$source" in
        git* | bitbucket:* | bb:* | gh:* | gl:*)
            ici_import_repository "$sourcespace" "$source"
            ;;
        http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
            ici_import_url "$sourcespace" "$source"
            ;;
        -.)
            local file; file=$(basename "$basepath")
            echo "Removing '${sourcespace:?}/$file'"
            rm -r "${sourcespace:?}/$file"
            ;;
        -*)
            local file="${source:1}"
            if [ ! -e "${sourcespace:?}/$file" ]; then
              file="$(basename "$basepath")/$file"
            fi
            echo "Removing '${sourcespace:?}/$file'"
            rm -r "${sourcespace:?}/$file"
            ;;
        .)
            echo "Copying '$basepath'"
            ici_import_directory "$sourcespace" "$basepath"
            ;;
        /*)
            if [ -d "$source" ]; then
                echo "Copying '$source'"
                ici_import_directory  "$sourcespace" "$source"
            elif [ -f "$source" ]; then
                ici_import_file "$sourcespace" "$source"
            else
                ici_error "'$source' cannot be found"
            fi
            ;;
        "")
            ici_error "source is empty string"
            ;;
        *)
            if [ -d "$basepath/$source" ]; then
                echo "Copying '$source'"
                ici_import_directory "$sourcespace" "$basepath/$source"
            elif [ -f "$basepath/$source" ]; then
                ici_import_file "$sourcespace" "$basepath/$source"
            else
                ici_error "cannot read source from '$source'"
            fi
            ;;
        esac
    done
}

function ici_setup_rosdep {
    ici_install_pkgs_for_command rosdep "${PYTHON_VERSION_NAME}-rosdep"
    ici_install_pkgs_for_command "pip${ROS_PYTHON_VERSION}" "${PYTHON_VERSION_NAME}-pip"

    if [ "$ROS_DISTRO" = "indigo" ] || [ "$ROS_DISTRO" = "jade" ]; then
        ici_quiet ici_apt_install "ros-$ROS_DISTRO-roslib"
    else
        ici_apt_install "ros-$ROS_DISTRO-ros-environment"
    fi

    # Setup rosdep
    rosdep --version
    if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
        ici_asroot rosdep init
    fi

    update_opts=()
    if [ -z "${ROSDISTRO_INDEX_URL:-}" ]; then
        update_opts+=(--rosdistro "$ROS_DISTRO")
    fi
    if [ "$ROS_VERSION_EOL" = true ]; then
        update_opts+=(--include-eol-distros)
    fi

    ici_retry 2 rosdep update "${update_opts[@]}"
}

function ici_exec_in_workspace {
    local extend=$1; shift
    local path=$1; shift
    ( { [ ! -e "$extend/setup.bash" ] || ici_source_setup "$extend"; } && cd "$path" && exec "$@")
}

function ici_install_dependencies {
    local extend=$1; shift
    local skip_keys=$1; shift

    local cmake_prefix_path=
    if [ "$ROS_VERSION" -eq 2 ]; then
      # work-around for https://github.com/ros-infrastructure/rosdep/issues/724
      cmake_prefix_path="$(ici_exec_in_workspace "$extend" . env | grep -oP '^CMAKE_PREFIX_PATH=\K.*'):" || true
    fi

    rosdep_opts=(-q --from-paths "$@" --ignore-src -y)
    if [ -n "$skip_keys" ]; then
      rosdep_opts+=(--skip-keys "$skip_keys")
    fi
    set -o pipefail # fail if rosdep install fails
    ROS_PACKAGE_PATH="$cmake_prefix_path${ROS_PACKAGE_PATH:-}" ici_exec_in_workspace "$extend" "." rosdep install "${rosdep_opts[@]}" | { grep "executing command" || true; }
    set +o pipefail
}

function ici_build_workspace {
    local name=$1; shift
    local extend=$1; shift
    local ws=$1; shift

    local ws_sources=()
    ici_parse_env_array  ws_sources "${name^^}_WORKSPACE"
    local sources=("$@" "${ws_sources[@]}")
    local cmake_args ws_cmake_args=()
    ici_parse_env_array  cmake_args CMAKE_ARGS
    ici_parse_env_array  ws_cmake_args "${name^^}_CMAKE_ARGS"
    local args=()
    if [ ${#cmake_args[@]} -gt 0 ] || [ ${#ws_cmake_args[@]} -gt 0 ]; then
        args+=(--cmake-args "${cmake_args[@]}" "${ws_cmake_args[@]}")
    fi

    ici_run "setup_${name}_workspace" ici_prepare_sourcespace "$ws/src" "${sources[@]}"
    ici_run "install_${name}_dependencies" ici_install_dependencies "$extend" "$ROSDEP_SKIP_KEYS" "$ws/src"
    ici_run "build_${name}_workspace" builder_run_build "$extend" "$ws" "${args[@]}"
}

function ici_test_workspace {
    local name=$1; shift
    local extend=$1; shift
    local ws=$1; shift

    ici_run "run_${name}_test" builder_run_tests "$extend" "$ws"
    builder_test_results "$extend" "$ws"
}
