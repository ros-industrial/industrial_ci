#!/bin/bash

# Copyright (c) 2020, Felix Messmer
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

function run_pylint_check() {
  ici_require_run_in_docker # this script must be run in docker
  target_ws=~/target_ws

  local -a sources
  ici_parse_env_array sources TARGET_WORKSPACE
  ici_run "prepare_sourcespace" ici_prepare_sourcespace "$target_ws/src" "${sources[@]}"

  local -a pylint_versions
  ici_parse_env_array pylint_versions PYLINT_VERSIONS
  local -a pylint_args
  ici_parse_env_array pylint_args PYLINT_ARGS

  for cmd in "${pylint_versions[@]}"; do
      ici_run "install_$cmd" ici_install_pkgs_for_command "$cmd" "$cmd"
      ici_with_ws "$target_ws" ici_run "run_$cmd" "$cmd" "${pylint_args[@]}" "$(find "$TARGET_REPO_PATH" -iname "*.py")"
  done
}
