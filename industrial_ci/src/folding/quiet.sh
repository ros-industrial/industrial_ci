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
    ici_error "ici_start_fold is not implemented"
}

function  ici_end_fold() {
    ici_error "ici_end_fold is not implemented"
}

function ici_report_result() {
    true
}

function ici_cmd {
    _ici_guard ici_label ici_quiet "$@"
}

function ici_filter {
    shift
    ici_quiet "$@"
}

function ici_step {
    shift
    _ici_guard ici_quiet  "$@"
}
