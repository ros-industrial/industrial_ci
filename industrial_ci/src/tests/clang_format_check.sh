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

function run_clang_format_check() {
  local err=0
  local path=$TARGET_REPO_PATH

  DOCKER_IMAGE="$DOCKER_BASE_IMAGE" ici_require_run_in_docker # this script must be run in docker

  # Check whether a specific version of clang-format is desired
  local clang_format_executable="clang-format${CLANG_FORMAT_VERSION:+-$CLANG_FORMAT_VERSION}"

  ici_time_start install_clang_format
  ici_quiet ici_asroot apt-get install -qq -y git-core "$clang_format_executable"
  ici_time_end # install_clang_format

  if [ -n "$USE_MOCKUP" ]; then
    if [ ! -d "$TARGET_REPO_PATH/$USE_MOCKUP" ]; then
        ici_error "mockup directory '$USE_MOCKUP' does not exist"
    fi
    path="$TARGET_REPO_PATH/$USE_MOCKUP"
  fi

  ici_time_start run_clang_format_check
  while read -r file; do
    if ! $clang_format_executable -style="$CLANG_FORMAT_CHECK" "$file" | git diff --exit-code "$file" - ; then
      err=$((err +1))
    fi
  done < <(find "$path"/* -iname '*.h' -or -iname '*.hpp' -or -iname '*.c' -or -iname '*.cc' -or -iname '*.cpp' -or -iname '*.cxx')

  if [ "$err" -ne "0" ]; then
      ici_error "Clang format check failed for $err file(s)."
      echo "Changes required to comply to formatting rules. See diff above."
      exit 1
  fi
  echo 'Clang format check went successful.'
  ici_time_end # run_clang_format_check
}
