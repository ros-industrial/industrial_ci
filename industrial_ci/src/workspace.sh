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
export _ROS_KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg

function ici_parse_repository_url {
    local url=$1; shift
    if [[ $url =~ ([^:]+):([^#]+)#(.+) ]]; then
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
            'git+file'*|'git+http'*|'git+ssh'*)
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
    ici_cmd ici_quiet ici_filter "Setting up" ici_asroot apt-get -qq install -y --no-upgrade --no-install-recommends "$@"
}

function ici_pip_install {
    ici_cmd ici_asroot "${PYTHON_VERSION_NAME}" -m pip install -q "$@"
}

function ici_process_url {
    local url=$1; shift
    ici_install_pkgs_for_command wget wget
    ici_guard wget -O- -q "$url" | ici_guard "$@"
}

function ici_gpg {
    local homedir
    ici_make_temp_dir homedir
    local keyring=$1
    shift
    ici_guard ici_asroot touch "$keyring"
    ici_cmd ici_asroot gpg --homedir "$homedir" --no-auto-check-trustdb --trust-model always --no-options --no-default-keyring --secret-keyring /dev/null --keyring "$keyring" "$@"
}

function ici_gpg_import {
    local keyring=$1
    shift
    ici_guard gpg --dearmor | >/dev/null ici_guard ici_asroot tee "$keyring"
    ici_cmd gpg --no-options --trust-model always --no-default-keyring --keyring "$keyring" --fingerprint
}

function ici_setup_gpg_key {
    case "$ROS_REPOSITORY_KEY}" in
    *://*)
        ici_process_url "${ROS_REPOSITORY_KEY}" ici_gpg_import "$_ROS_KEYRING"
        ;;
    /*.*)
        ici_gpg_import "$_ROS_KEYRING" < "$ROS_REPOSITORY_KEY"
        ;;
    *.*)
        ici_gpg_import "$_ROS_KEYRING" < "$TARGET_REPO_PATH/$ROS_REPOSITORY_KEY"
        ;;
    *)
        if [[ $ROS_REPOSITORY_KEY =~ ^[a-fA-F0-9]{16,40}$ ]]; then
            local sks=()
            if [ -n  "${APTKEY_STORE_SKS}" ]; then
                sks=(--keyserver "${APTKEY_STORE_SKS}")
            fi
            ici_retry 3 ici_gpg "$_ROS_KEYRING" "${sks[@]}"  --recv-key "${ROS_REPOSITORY_KEY}"
        else
            ici_error "Cannot classify ROS_REPOSITORY_KEY '$ROS_REPOSITORY_KEY'"
        fi
        ;;
    esac
}

function ici_init_apt {
    if [ -n "$APT_PROXY" ]; then
      echo "Acquire::http::Proxy \"$APT_PROXY\";" | >/dev/null ici_asroot tee /etc/apt/apt.conf.d/99-industrial_ci-proxy
    fi
    echo 'debconf debconf/frontend select Noninteractive' |  ici_quiet ici_asroot debconf-set-selections

    ici_cmd ici_asroot sed -i "/^# deb.*multiverse/ s/^# //" /etc/apt/sources.list

    if 2>/dev/null apt-key adv -k C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 | grep -q expired; then
        ici_warn "Expired ROS repository key found, installing new one"
        ici_retry 3 ici_cmd apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    fi

    ici_cmd ici_asroot apt-get update -qq

    local debs_default=(apt-utils build-essential gnupg2 dirmngr)
    if ! ls /etc/ssl/certs/* 2&> /dev/null; then
        debs_default+=(ca-certificates)
    fi
    if [ -n "$_DEFAULT_DEBS" ]; then
        ici_parse_env_array debs_default _DEFAULT_DEBS
    fi
    if [ -n "${debs_default[*]}" ]; then
        ici_apt_install "${debs_default[@]}"
    fi

    if [ -n "$ROS_DISTRO" ]; then
        local current_repository_path
        current_repository_path="$(apt-cache policy "ros-$ROS_DISTRO-ros-core" 2> /dev/null | grep -Eo "[^ ]+://[^ ]+")" || true
        ici_set_ros_repository_path "$current_repository_path"
    fi

    if [ -n "${ROS_REPOSITORY_PATH:-}" ] && ! grep -qFs "$ROS_REPOSITORY_PATH" /etc/apt/sources.list.d/*.list; then
        if [ -n "$current_repository_path" ] && [ "$current_repository_path" != "$ROS_REPOSITORY_PATH" ]; then
            ici_warn "Setting up repository '$ROS_REPOSITORY_PATH' next to '$current_repository_path', please double check ROS_REPO='$ROS_REPO'"
        fi
        ici_install_pkgs_for_command lsb_release lsb-release
        local deb_opts=(arch="$(dpkg --print-architecture)" signed-by="$_ROS_KEYRING")
        ici_setup_gpg_key

        >/dev/null ici_asroot tee "/etc/apt/sources.list.d/ros${ROS_VERSION}-latest.list" <<< "deb [${deb_opts[*]}] ${ROS_REPOSITORY_PATH} $(lsb_release -sc) main"
        ici_cmd ici_asroot apt-get update -qq
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
    ici_guard vcs import --recursive --force "$@"
}

function ici_import_repository {
    local sourcespace=$1; shift
    local url=$1; shift

    ici_install_pkgs_for_command vcs python3-vcstool

    local parsed; parsed=$(ici_parse_repository_url "$url")
    IFS=" " read -r -a parts <<< "$parsed" # name, type, url, version
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
        ici_guard bsdtar -C "$sourcespace" -xf "$file"
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
    ici_guard ici_process_url "$url" "${processor[@]}"
}

function  ici_import_directory {
    local sourcespace=$1; shift
    local dir=$1; shift
    local target
    target=${sourcespace:?}/$(basename "$dir")

    ici_guard rm -rf "$target"
    ici_guard mkdir "$target"
    local args=()
    for p in "$BASEDIR" "$CCACHE_DIR" "$(readlink -m "$ICI_SRC_PATH/../..")"; do
        if [[ $p/ ==  $dir/?* ]]; then
            args+=("--exclude=.${p#"$dir"}")
        fi
    done
    ici_guard tar c "${args[@]}" -C "$dir" . | ici_guard tar x -C "$target"
}

function ici_prepare_sourcespace {
    local sourcespace=$1; shift
    local basepath=$TARGET_REPO_PATH

    ici_guard mkdir -p "$sourcespace"

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
            ici_log "Removing '${sourcespace:?}/$file'"
            ici_guard rm -r "${sourcespace:?}/$file"
            ;;
        -*)
            local file="${source:1}"
            if [ ! -e "${sourcespace:?}/$file" ]; then
              file="$(basename "$basepath")/$file"
            fi
            ici_log "Removing '${sourcespace:?}/$file'"
            ici_guard rm -r "${sourcespace:?}/$file"
            ;;
        .)
            ici_log "Copying '$basepath'"
            ici_import_directory "$sourcespace" "$basepath"
            ;;
        /*)
            if [ -d "$source" ]; then
                ici_log "Copying '$source'"
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
                ici_log "Copying '$source'"
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
        ici_apt_install "ros-$ROS_DISTRO-roslib"
    else
        ici_apt_install "ros-$ROS_DISTRO-ros-environment"
    fi

    if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
        ici_cmd ici_quiet ici_asroot rosdep init
    fi
    if [ -n "$ROSDEP_SOURCES_VERSION" ]; then
        ici_cmd ici_quiet ici_asroot sed -E -i "s;(https://raw.githubusercontent.com/ros/rosdistro/)master;\1$ROSDEP_SOURCES_VERSION;g" /etc/ros/rosdep/sources.list.d/*.list
    fi

    update_opts=()
    if [ -z "${ROSDISTRO_INDEX_URL:-}" ]; then
        update_opts+=(--rosdistro "$ROS_DISTRO")
    fi
    if [ "$ROS_VERSION_EOL" = true ]; then
        update_opts+=(--include-eol-distros)
    fi

    ici_retry 2 ici_cmd ici_quiet rosdep update "${update_opts[@]}"
}

function ici_extend_space {
  echo "$1/install"
}

function ici_exec_in_workspace {
    local extend=$1; shift
    local path=$1; shift
    ( ici_source_setup "$extend" && cd "$path" && exec "$@") || ici_exit
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

    ROS_PACKAGE_PATH="$cmake_prefix_path${ROS_PACKAGE_PATH:-}" ici_cmd ici_quiet ici_filter "(executing command)|(Setting up)" ici_exec_in_workspace "$extend" "." rosdep install "${rosdep_opts[@]}"
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

    ici_step "setup_${name}_workspace" ici_prepare_sourcespace "$ws/src" "${sources[@]}"
    ici_step "install_${name}_dependencies" ici_install_dependencies "$extend" "$ROSDEP_SKIP_KEYS" "$ws/src"
    ici_step "build_${name}_workspace" builder_run_build "$extend" "$ws" "${args[@]}"
}

function ici_test_workspace {
    local name=$1; shift
    local extend=$1; shift
    local ws=$1; shift
    local err=0

    ici_step "run_${name}_test" builder_run_tests "$extend" "$ws"
    builder_test_results "$extend" "$ws" || err=$?
    ici_report_result "${name}_test_results" "$err"
    return "$err"
}

function ici_source_setup {
    local extend=$1; shift
    if  [ ! -f "$extend/setup.bash" ]; then
        if [ "$extend" != "/opt/ros/$ROS_DISTRO" ]; then
            ici_error "'$extend' is not a devel/install space"
        fi
    else
        ici_with_unset_variables source "$extend/setup.bash"
        if [ "$ROS_VERSION" -eq 1 ] && [ -f "$extend/.colcon_install_layout" ]; then
            # Fix for https://github.com/ros-industrial/industrial_ci/issues/624
            # (Re)populate ROS_PACKAGE_PATH from all underlays
            # shellcheck disable=SC1090
            source "/opt/ros/$ROS_DISTRO/etc/catkin/profile.d/1.ros_package_path.sh"
        fi
    fi
}

function ici_with_ws() {
    local err=0
    # shellcheck disable=SC2034
    current_ws=$1; shift
    "$@" || err=$?
    unset current_ws
    return "$err"
}
