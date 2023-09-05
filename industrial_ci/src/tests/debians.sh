#!/bin/bash

# Copyright (c) 2021, Mathias Lüdtke
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

# Based on https://github.com/ros-industrial/industrial_ci/issues/697#issuecomment-876293987


function make_repo() {
    local repo=$1; shift
    ici_guard rm -rf "$repo"
    ici_guard mkdir -p "$repo"
    ici_guard touch "$repo/Packages"
}

function update_repo() (
    local repo=$1; shift
    ici_guard apt-ftparchive packages "$repo" > "$repo/Packages"
    ici_guard apt-ftparchive release -o APT::FTPArchive::Release::Origin=industrial_ci "$repo" > "$repo/Release"
)

function use_repo() (
    local repo=$1; shift
    ici_guard ici_asroot rm -f /etc/apt/apt.conf.d/docker-clean
    echo "deb [trusted=yes] file://$repo ./" | >/dev/null ici_guard ici_asroot tee /etc/apt/sources.list.d/ici_debians.list
    echo -e 'Package: *\nPin: release o=industrial_ci\nPin-Priority: 1000' | >/dev/null ici_guard ici_asroot tee /etc/apt/preferences.d/ici_debians
    ici_guard ici_asroot apt-get -o APT::Sandbox::User=root update -qq
)

function forward_mounts() {
    ici_forward_mount WORKSPACE rw
    ici_forward_mount "$WORKSPACE/archives" rw /var/cache/apt/archives
    if [ -n "${DOCKER_PORT:-}" ]; then
        ici_forward_variable DOCKER_HOST "$DOCKER_PORT"
    elif [ -e /var/run/docker.sock ]; then
        ici_forward_mount /var/run/docker.sock rw
    fi
}

function prepare_debians() {
    export WORKSPACE; WORKSPACE=$(mktemp -d)
    ici_guard mkdir "$WORKSPACE/archives"
    forward_mounts
}

function build_debian() (
    local pkg_path=$1; shift
    local repo=$1; shift
    use_repo "$repo"

    ici_guard cd "$pkg_path"
    ici_cmd ici_filter "Setting up" ici_asroot apt-get build-dep -y -qq .
    ici_cmd ici_quiet dpkg-buildpackage -b -uc -us
    ici_guard mv ../*.deb "$repo"
)

function isolate_build_debian() (
    local pkg_path=$1; shift
    local repo=$1; shift
    for hook in $(env | grep -o '^\(BEFORE\|AFTER\)_[^=]*'); do
        unset "$hook"
    done
    export _FOLDING_TYPE=quiet
    DOCKER_PULL=false ici_isolate debians build_debian "$pkg_path" "$repo" || ici_exit
)

function build_package() {
    local pkg_path=$1; shift
    local repo=$1; shift

    use_repo "$repo"
    ici_install_pkgs_for_command dpkg-buildpackage dpkg-dev

    ( ici_guard cd "$pkg_path" && ici_cmd bloom-generate rosdebian --ros-distro="$ROS_DISTRO" --debian-inc="ici~"; ) || ici_exit
    # https://github.com/ros-infrastructure/bloom/pull/643
    echo 11 > "$pkg_path"/debian/compat

    isolate_build_debian "$pkg_path" "$repo" || ici_exit
    update_repo "$repo"
}

function test_install_packages() {
    local repo=$1; shift
    ( ici_guard cd "$repo" && ici_apt_install ./*.deb; )
}

function run_debians() {
    ici_guard source "${ICI_SRC_PATH}/isolation/docker.sh"
    forward_mounts

    export BUILDER=colcon
    ici_source_builder
    ici_step "${BUILDER}_setup" builder_setup
    ici_step "setup_bloom" ici_install_pkgs_for_command bloom-generate python3-bloom debhelper
    ici_step "setup_apt_utils" ici_install_pkgs_for_command apt-ftparchive apt-utils
    ici_step "setup_docker" ici_install_pkgs_for_command docker docker.io
    ici_step "setup_rosdep" ici_setup_rosdep

    local repo="$WORKSPACE/repository"
    make_repo "$repo"

    for name in upstream target downstream; do
      local sources=()
      local current="$WORKSPACE/${name}_ws/src"
      ici_parse_env_array sources "${name^^}_WORKSPACE"
      if [ -n "${sources[*]}" ]; then
        ici_step "prepare_${name}_sourcespace" ici_prepare_sourcespace "$current" "${sources[@]}"

        while read -r -a pkg; do
            ici_step "build_${pkg[0]}" build_package "$current/${pkg[1]}" "$repo"
        done < <(ici_guard cd "$current" && ici_guard colcon list -t)
      fi
    done

    ici_step "test_install_packages" test_install_packages "$repo"

    ici_log "Debian packages:"
    ici_redirect find "$repo" -name '*.deb' -exec basename {} \;
}
