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

function _install_universal_ctags() {
    ici_apt_install autoconf automake pkg-config
    ici_import_repository /tmp github:universal-ctags/ctags.git#master
    (ici_guard cd /tmp/ctags && ./autogen.sh && ./configure && ici_asroot make install) || ici_exit
    rm -rf /tmp/ctags
}

function _import_and_make_install() {
  local repo=$1; shift
  ici_import_repository /tmp "github:$repo.git#master"
  local dir; dir=/tmp/$(basename "$repo")
  (ici_guard cd "$dir" && ici_asroot make install prefix=/usr) || ici_exit
  rm -rf "$dir"
}

function _install_vtable_dumper() {
    ici_apt_install libelf-dev make
    _import_and_make_install lvc/vtable-dumper
}

function _install_abi_dumper() {
    ici_apt_install binutils elfutils perl make
    _import_and_make_install lvc/abi-dumper
}

function _install_abi_compliance_checker() {
    ici_apt_install perl make
    _import_and_make_install lvc/abi-compliance-checker
}

function abi_install_dumper() {
  ici_exec_for_command vtable-dumper ici_quiet _install_vtable_dumper
  ici_exec_for_command abi-dumper ici_quiet _install_abi_dumper
  ici_exec_for_command ctags ici_quiet _install_universal_ctags
}

function abi_install_compliance_checker() {
  ici_exec_for_command abi-compliance-checker ici_quiet _install_abi_compliance_checker
}

function abi_dump_libraries() {
    abi_install_dumper

    local extend=$1; shift
    local output=$1; shift

    local ld_library_path
    ld_library_path=$(ici_source_setup "$extend" && echo "$LD_LIBRARY_PATH")

    mkdir -p "$output"
    for d in "$extend"/*/lib "$extend/lib"; do
      for l in "$d"/*.so; do
        if [ "$l" != "$d/*.so" ]; then
          abi-dumper "$l" -ld-library-path "$ld_library_path" -o "$output/$(basename "$l" .so).dump" -public-headers "$d/../include" "$@"
        fi
      done
    done
}

function abi_process_workspace() {
  local extend=$1; shift
  local workspace=$1; shift
  local tag=$1; shift
  local version=${1:-$tag}

  local cflags="-g -Og"
  local cmake_args=(--cmake-args "-DCMAKE_C_FLAGS=$cflags" "-DCMAKE_CXX_FLAGS=$cflags")

  ici_step "install_${tag}_dependencies" ici_install_dependencies "$extend" "$ROSDEP_SKIP_KEYS" "$workspace/src"
  ici_step "abi_build_${tag}" builder_run_build "$extend" "$workspace" "${cmake_args[@]}"
  ici_step "abi_dump_${tag}" abi_dump_libraries  "$(ici_extend_space "$workspace")" "$workspace/abi_dumps" -lver "$version"
}

function abi_configure() {
  if [ "$ABICHECK_MERGE" = true ]; then
    ici_exec_for_command git ici_error "ABICHECK_MERGE=true needs git client"
    local ref_list
    if ici_split_array ref_list "$(cd "$TARGET_REPO_PATH" && git rev-list --parents -n 1 HEAD)" && [ "${#ref_list[@]}" -gt 2 ]; then
        ABICHECK_VERSION="${ref_list[1]}"
        ABICHECK_URL="#${ABICHECK_VERSION}"
    else
        ici_error "Could not find merge commit for ABI check"
    fi
  fi

  if [ -z "$ABICHECK_VERSION" ]; then
    if [[ $ABICHECK_URL =~ [@#]([[:alnum:]_.-]+)$ ]]; then
        ABICHECK_VERSION=${BASH_REMATCH[1]}
    else
        local target_ext
        target_ext=$(grep -Pio '\.(zip|tar\.\w+|tgz|tbz2)\Z' <<< "${ABICHECK_URL%%\?*}" || echo "")
        if [ -n "$target_ext" ]; then
            ABICHECK_VERSION=$(basename "$ABICHECK_URL" "$target_ext")
        else
          ici_warn "could not determine ABICHECK_VERSION"
          ABICHECK_VERSION=unknown
        fi
    fi
  fi
}

function abi_report() {
  local base_dumps=$1; shift
  local target_dumps=$1; shift
  local reports_dir=$1; shift

  abi_install_compliance_checker
  ici_install_pkgs_for_command links links

  mkdir -p "$reports_dir"

  local broken=()
  for n in "$target_dumps"/*.dump; do
      local l; l=$(basename "$n" ".dump")
      local o="$base_dumps/$l.dump"
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
              return "$ret"
          fi
      fi
  done

  if [ "${#broken[@]}" -gt "0" ]; then
      ici_error "Broken libraries: ${broken[*]}"
  fi
}

 function prepare_abi_check() {
    if [ "$ABICHECK_MERGE" = "auto" ]; then
        ici_error "ABICHECK_MERGE auto mode is available for travis and github only. "
    fi
    if [ -z "$ABICHECK_URL" ]; then
        ici_error "Please set ABICHECK_URL"
    fi

    ici_check_builder

    abi_configure # handle merge and detect version
 }

function run_abi_check() {
    if [[ $ABICHECK_URL =~ ^[@#]([[:alnum:]_.-]+)$ ]]; then
        ABICHECK_URL="git+file://${TARGET_REPO_PATH}${ABICHECK_URL}"
    fi

    base_ws=$BASEDIR/${PREFIX}base_ws
    upstream_ws=$BASEDIR/${PREFIX}upstream_ws
    target_ws=$BASEDIR/${PREFIX}target_ws

    ici_with_ws "$base_ws" ici_step "abi_get_base" ici_prepare_sourcespace "$base_ws/src" "$ABICHECK_URL"

    ici_source_builder
    ici_step "${BUILDER}_setup" builder_setup
    ici_step "setup_rosdep" ici_setup_rosdep

    if [ -n "$CCACHE_DIR" ]; then
        ici_step "setup_ccache" ici_apt_install ccache
        export PATH="/usr/lib/ccache:$PATH"
    fi

    extend=${UNDERLAY:?}

    if [ -n "$UPSTREAM_WORKSPACE" ]; then
        ici_with_ws "$upstream_ws" ici_build_workspace "upstream" "$extend" "$upstream_ws"
        extend="$(ici_extend_space "$upstream_ws")"
    fi

    mkdir -p "$target_ws/src"
    ici_import_directory "$target_ws/src" "$TARGET_REPO_PATH"

    ici_step "abi_install_compliance_checker" abi_install_compliance_checker
    ici_step "abi_install_dumper" abi_install_dumper

    ici_with_ws "$target_ws" abi_process_workspace "$extend" "$target_ws" target
    ici_with_ws "$base_ws" abi_process_workspace "$extend" "$base_ws" base "$ABICHECK_VERSION"

    abi_report "$base_ws/abi_dumps" "$target_ws/abi_dumps" "$BASEDIR/abicheck/$ABICHECK_VERSION"
}
