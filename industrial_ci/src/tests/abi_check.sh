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

function abi_install() {
    ici_asroot apt-get update -qq
    ici_asroot apt-get install -y -qq libelf-dev elfutils autoconf pkg-config links bsdtar wget

    wget -q -O /tmp/abi_installer.pl https://raw.githubusercontent.com/lvc/installer/master/installer.pl
    ici_asroot perl /tmp/abi_installer.pl -install -prefix /usr abi-compliance-checker
    ici_asroot perl /tmp/abi_installer.pl -install -prefix /usr abi-dumper

    git clone --depth 1 https://github.com/universal-ctags/ctags.git /tmp/ctags
    (cd /tmp/ctags && ./autogen.sh && ./configure && ici_asroot make install)
}


function abi_prepare_src() {
    local target_dir=$1
    local url=$2

    mkdir -p "$target_dir"

    if [[ "$url" =~ ^#(.*)$ ]]; then
        cp -a "$TARGET_REPO_PATH" "$target_dir"
        (cd "$target_dir/$(basename "$TARGET_REPO_PATH")" && git checkout "${BASH_REMATCH[1]}")
    else
        echo "Download '$url'"
        set -o pipefail
        wget -q -O - "$url" | bsdtar -C "$target_dir" -xf-
        set +o pipefail
    fi
}

function abi_build_workspace() {
    local base=$1
    local version=$2
    local workspace="$base/$version"

    local cflags="-g -Og"

    rosdep install -q --from-paths "$workspace/src" --ignore-src -y

    catkin config --init --install -w "$workspace" --extend "/opt/ros/$ROS_DISTRO" --cmake-args -DCMAKE_C_FLAGS="$cflags" -DCMAKE_CXX_FLAGS="$cflags"
    catkin build -w "$workspace"

    mkdir "$workspace/abi_dumps"
    for l in "$workspace/install/lib"/*.so; do
        abi-dumper "$l" -ld-library-path "/opt/ros/$ROS_DISTRO/lib" -o "$workspace/abi_dumps/$(basename "$l" .so).dump" -lver "$version" -public-headers "$workspace/install/include"
    done
}

function abi_setup_rosdep() {
    # Setup rosdep
    rosdep --version
    if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
        ici_asroot rosdep init
    fi

    update_opts=()
    if [ "$ROS_VERSION_EOL" = true ] && rosdep update --help | grep -q -- --include-eol-distros; then
      update_opts+=(--include-eol-distros)
    fi

    ici_retry 2 rosdep update "${update_opts[@]}"
}

function run_abi_check() {
    export ABICHECK_VERSION
    export ABICHECK_URL

    if [ "$ABICHECK_MERGE" = "auto" ]; then
        ici_error "ABICHECK_MERGE auto mode is available for travis only. "
    fi
    if [ -z "$ABICHECK_URL" ]; then
        ici_error "Please set ABICHECK_URL"
    fi

    if [[ $ABICHECK_URL =~ ([^:]+):([^#]+)#(.+) ]]; then
        ABICHECK_VERSION="${BASH_REMATCH[3]}"
        case "${BASH_REMATCH[1]}" in
            bitbucket)
                ABICHECK_URL="https://bitbucket.org/${BASH_REMATCH[2]}/get/$ABICHECK_VERSION.tar.gz"
                ;;
            github)
                ABICHECK_URL="https://github.com/${BASH_REMATCH[2]}/archive/$ABICHECK_VERSION.tar.gz"
                ;;
            gitlab)
                ABICHECK_URL="https://gitlab.com/${BASH_REMATCH[2]}/repository/$ABICHECK_VERSION/archive.tar.gz"
                ;;
            *)
                ici_error "scheme '${BASH_REMATCH[1]}' is not supported for ABICHECK_URL"
        esac
    fi

    if [ "$ABICHECK_MERGE" = true ]; then
      local ref_list
      if ref_list=($(cd "$TARGET_REPO_PATH" && git rev-list --parents -n 1 HEAD)) && [ "${#ref_list[@]}" -gt 2 ]; then
          ABICHECK_URL="#${ref_list[1]}"
          ABICHECK_VERSION="${ref_list[1]}"
      else
          ici_error "Could not find merge commit for ABI check"
      fi
    fi

    if [ -z "$ABICHECK_VERSION" ]; then
        local target_ext
        target_ext=$(grep -Pio '\.(zip|tar\.\w+|tgz|tbz2)\Z' <<< "${ABICHECK_URL%%\?*}" || echo "")
        ABICHECK_VERSION=$(basename "$ABICHECK_URL" "$target_ext")
    fi

    ici_require_run_in_docker # this script must be run in docker

    ici_time_start install_abi_compliance_checker
    ici_quiet abi_install
    ici_time_end  # install_abi_compliance_checker

    ici_time_start abi_get_base
    abi_prepare_src "/abicheck/old/$ABICHECK_VERSION/src" "$ABICHECK_URL"
    ici_time_end  # abi_get_base

    ici_time_start setup_rosdep
    abi_setup_rosdep
    ici_time_end  # setup_rosdep

    ici_time_start abi_build_new
    mkdir -p "/abicheck/new/src"
    ln -s "$TARGET_REPO_PATH" "/abicheck/new/src"
    abi_build_workspace /abicheck new
    ici_time_end  # abi_build_new

    ici_time_start abi_build_base
    abi_build_workspace /abicheck/old "$ABICHECK_VERSION"
    ici_time_end  # abi_build_base

    local reports_dir="/abicheck/reports/$ABICHECK_VERSION"
    mkdir -p "$reports_dir"

    local broken=()
    for n in /abicheck/new/abi_dumps/*.dump; do
        local l; l=$(basename "$n" ".dump")
        local o="/abicheck/old/$ABICHECK_VERSION/abi_dumps/$l.dump"
        if [ -f "$o" ]; then
            ici_time_start "abi_check_$l"
            local ret=0
            abi-compliance-checker -report-path "$reports_dir/$l.html" -l "$l" -n "$n" -o "$o" || ret=$?
            if [ "$ret" -eq "0" ]; then
                ici_time_end # abi_check_*
            elif [ "$ret" -eq "1" ]; then
                links -dump "$reports_dir/$l.html"
                broken+=("$l")
                ici_time_end "${ANSI_YELLOW}" "$ret" # abi_check_*, yellow
            elif [ "$ret" -eq "7" ]; then
                ici_warn "'$(basename "$l" .dump)': Invalid input ABI dump. Perhaps this library does not any export symbols."
                ici_time_end "${ANSI_YELLOW}"
            else
                ici_exit "$ret"
            fi
        fi
    done

    if [ "${#broken[@]}" -gt "0" ]; then
        ici_error "Broken libraries: ${broken[*]}"
    fi
}
