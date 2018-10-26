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

function clang_install() {
  local version="$1"
  sudo apt-get update -qq
  sudo apt-get install -y -qq clang-format"$version"
}

function run_clang_format_check() {
  local ERR=0

  ici_require_run_in_docker # this script must be run in docker
  
  ici_time_start clang_install
    clang_install "$version"
  ici_time_end # clang_install
  
  ici_time_start run_clang_format_check
  while read file; do
    if ! clang-format -style="$CLANG_FORMAT_CHECK" "$file" | git diff --exit-code "$file" - ; then
      ERR=$[$ERR +1]
    fi
  done < <(find "$TARGET_REPO_PATH" -name '*.h' -or -name '*.hpp' -or -name '*.cpp') 
  
  if [ "$ERR" -ne "0" ]; then
      echo "Clang format check failed for $ERR file(s)."
      echo "Changes required to comply to formatting rules. See diff above."
      exit 1
  fi
  echo 'Clang format check went successful.'
  ici_time_end # run_clang_format_check
}