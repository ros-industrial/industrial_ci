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

    export TRAVIS_BUILD_DIR=$(mktemp -d)

    repo_name=${_EXTERNAL_REPO%%#*}
    repo_branch=${_EXTERNAL_REPO##*#}

    if [ "$repo_branch" = "$_EXTERNAL_REPO" ]; then # if branch name was not provided
        repo_branch="HEAD"
    fi

    if [[ "$repo_name" =~ ^[a-zA-Z][\w+-\.]*: ]]; then # stars with scheme (RFC 3986)
        repo_url=$repo_name
    else
        repo_url="https://github.com/$repo_name.git" # treat as github repo
    fi

    git clone "$repo_url" "$TRAVIS_BUILD_DIR"
    git -C "$TRAVIS_BUILD_DIR" checkout "$repo_branch"

    urlbasename=${repo_url##*/}
    urldirname=${repo_url%/$urlbasename}
    export TRAVIS_REPO_SLUG="${urldirname##*/}/${urlbasename%.git}"
fi

./travis.sh
