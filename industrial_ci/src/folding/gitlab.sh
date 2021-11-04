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
    shift
    local name=$1; shift
    local start=$1; shift
    ici_ansi_cleared_line "section_start:${start::-9}:${name}[collapsed=true]"
}

function  ici_end_fold() {
    shift
    local name=$1; shift
    shift
    local end=$1; shift
    ici_ansi_cleared_line "section_end:${end::-9}:$name"
}

function ici_report_result() {
    true
}
