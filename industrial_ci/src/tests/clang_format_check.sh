#!/bin/bash

# Copyright (c) 2018, Jonathan Hechtbauer
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

function prepare_clang_format_check() {
  true
}

function run_clang_format_check() {
  local err=0
  local path
  ici_make_temp_dir path

  # Check whether a specific version of clang-format is desired
  local clang_format_executable="clang-format${CLANG_FORMAT_VERSION:+-$CLANG_FORMAT_VERSION}"

  ici_time_start install_clang_format

  # Install llvm repository to install the correct clang version if not supported by default on the distro
  if ! apt-cache search --names-only "$clang_format_executable" | grep -q "clang"; then
    ici_install_pkgs_for_command wget wget
    ici_apt_install lsb-release software-properties-common gnupg
    ici_cmd wget -qO /tmp/llvm.sh https://apt.llvm.org/llvm.sh
    ici_cmd chmod +x /tmp/llvm.sh
    ici_cmd ici_asroot /tmp/llvm.sh "$CLANG_FORMAT_VERSION"
  fi

  ici_apt_install git-core "$clang_format_executable"
  ici_time_end # install_clang_format

  local sources=()
  ici_parse_env_array sources TARGET_WORKSPACE
  ici_step "prepare_sourcespace" ici_prepare_sourcespace "$path" "${sources[@]}"

  ici_time_start run_clang_format_check
  while read -r file; do
    ici_log "Checking '${file#"$path/"}'"
    if ! $clang_format_executable -style="$CLANG_FORMAT_CHECK" "$file" | git diff --exit-code "$file" - ; then
      err=$((err +1))
    fi
  done < <(ici_find_nonhidden "$path" -iname '*.h' -or -iname '*.hpp' -or -iname '*.c' -or -iname '*.cc' -or -iname '*.cpp' -or -iname '*.cxx')

  if [ "$err" -ne "0" ]; then
      ici_error "Clang format check failed for $err file(s)."
  fi
  ici_log 'Clang format check went successful.'
  ici_time_end # run_clang_format_check
}
