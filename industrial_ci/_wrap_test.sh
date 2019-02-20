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

# This is the internal entrypoint for industrial_ci testing

set -e # exit script on errors

if [ -n "$_EXTERNAL_REPO" ]; then
    export TRAVIS_BUILD_DIR; TRAVIS_BUILD_DIR=$(mktemp -d)
    source ./industrial_ci/src/workspace.sh
    IFS=" " read -r -a parts <<< "$(ici_resolve_scheme "$_EXTERNAL_REPO")" # name, type, url, version
    echo "Cloning '${parts[2]}'...'"
    git clone -q "${parts[2]}" "$TRAVIS_BUILD_DIR"
    git -C "$TRAVIS_BUILD_DIR" checkout "${parts[3]}"

    urlbasename=${parts[2]##*/}
    urldirname=${parts[2]%/$urlbasename}
    export TRAVIS_REPO_SLUG="${urldirname##*/}/${urlbasename%.git}"
fi

./travis.sh
