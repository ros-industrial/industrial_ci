#!/bin/bash

# Copyright (c) 2016, Isaac I. Y. Saito
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

# This is the entrypoint for Gitlab CI only.

# 2016/05/18 http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in
DIR_THIS="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export TARGET_REPO_PATH=$CI_PROJECT_DIR
export TARGET_REPO_NAME=$CI_PROJECT_NAME
export _FOLDING_TYPE=gitlab

if [ -n "$SSH_PRIVATE_KEY" ]; then
  if [ "$CI_DISPOSABLE_ENVIRONMENT" != true ] && ! [ -f /.dockerenv ] ; then
    echo "SSH auto set-up cannot be used in non-disposable environments"
    exit 1
  fi

  # start SSH agent
  # shellcheck disable=SC2046
  eval $(ssh-agent -s)
  # add key to agent
  ssh-add <(echo "$SSH_PRIVATE_KEY") || { res=$?; echo "could not add ssh key"; exit $res; }

  if [ -n "$SSH_SERVER_HOSTKEYS" ]; then
    mkdir -p ~/.ssh
    # setup known hosts
    echo "$SSH_SERVER_HOSTKEYS" > ~/.ssh/known_hosts
  fi
fi

env "$@" bash "$DIR_THIS/industrial_ci/src/ci_main.sh"
