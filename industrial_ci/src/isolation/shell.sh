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

# ici_forward_mount VARNAME/FILE rw/ro [PATH]
function ici_forward_mount() {
  true
}

# ici_forward_variable VARNAME [VALUE]
function ici_forward_variable() {
  if [ -n "${2-}" ]; then
    export "$1"="$2"
  fi
}

function ici_isolate {
  if [ "${CI:-}" != true ] ; then
    ici_error 'ISOLATION=shell needs CI=true'
  fi
  if [ -z "${ROS_DISTRO:-}" ]; then
    ici_error "ROS_DISTRO is not set"
  elif [ "${ROS_DISTRO}" = "false" ]; then
      unset ROS_DISTRO
  fi
  if [ -n "${BASEDIR-}" ]; then
    mkdir -p "$BASEDIR"
  fi
  "${ICI_SRC_PATH}/run.sh" "$@"
}
