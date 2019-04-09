#!/bin/bash

# Copyright (c) 2016, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Lüdtke
# Copyright (c) 2018, Alexander Rössler
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

# This is the entrypoint for BitBucket Pipelines only.

# 2016/05/18 http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in
DIR_THIS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export TARGET_REPO_PATH=$BITBUCKET_CLONE_DIR
export TARGET_REPO_NAME=${BITBUCKET_REPO_SLUG##*/}
export PYTHONUNBUFFERED=${PYTHONUNBUFFERED:1}
export _DO_NOT_FOLD=true

env "$@" bash "$DIR_THIS/industrial_ci/src/ci_main.sh"
