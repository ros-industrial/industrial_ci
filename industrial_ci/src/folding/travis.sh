#!/bin/bash

# Copyright (c) 2019, Mathias Lüdtke
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

function  ici_start_fold() {
    local tag=$1; shift
    local name=$1; shift
    local start=$1; shift
    ici_ansi_cleared_line "travis_fold:start:$name"
    ici_ansi_cleared_line "travis_time:start:$tag"

}

function  ici_end_fold() {
    local tag=$1; shift
    local name=$1; shift
    local start=$1; shift
    local end=$1; shift
    ici_ansi_cleared_line "travis_time:end:$tag:start=$start,finish=$end,duration=$((end - start))"
    ici_ansi_cleared_line "travis_fold:end:$name"
}
