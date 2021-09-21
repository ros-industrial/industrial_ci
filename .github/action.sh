#!/bin/bash

# Copyright (c) 2020, Mathias Lüdtke
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

# This is the entrypoint for GitHub Actions only.

# 2016/05/18 http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in

set -euo pipefail

DIR_THIS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export TARGET_REPO_PATH=$GITHUB_WORKSPACE
export TARGET_REPO_NAME=${GITHUB_REPOSITORY##*/}
export _FOLDING_TYPE=github_actions

ICI_SRC_PATH="$DIR_THIS/../industrial_ci/src"
source "$ICI_SRC_PATH/util.sh"

if [ -n "${INPUT_CONFIG-}" ]; then
    ici_exec_for_command jq ici_error "In order to use the config parameter, please install jq"
    vars=$(jq -r 'keys[] as $k | "export \($k)=\(.[$k]|tojson)" | gsub("\\$\\$";"\\$")' <<< "$INPUT_CONFIG"  | grep "^export [A-Z][A-Z_]*=")
    echo "$vars"
    eval "$vars"
fi

if [ "${ABICHECK_MERGE:-}" = "auto" ]; then
  export ABICHECK_MERGE=false
  [ "$GITHUB_EVENT_NAME" != "pull_request" ] || ABICHECK_MERGE=true
fi

if [ "${ACT:-}" = true ]; then
  export _BUNDLE_ICI=true
  _FOLDING_TYPE=none
  ici_warn "Detected act, bundling industrial_ci"
fi

env "$@" bash "$ICI_SRC_PATH/ci_main.sh"
