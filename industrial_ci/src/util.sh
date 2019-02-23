#!/bin/bash

# Originally developed in JSK travis package https://github.com/jsk-ros-pkg/jsk_travis

# Copyright (c) 2016, Isaac I. Y. Saito
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

## util.sh
## This is a script where the functions commonly used within the industrial_ci repo are defined.

ANSI_RED=31
ANSI_GREEN=32
ANSI_YELLOW=33
ANSI_BLUE=34

function ici_color_output {
  local c=$1
  shift
  echo -e "\e[${c}m$*\e[0m"
}

function rosenv() (
  # if current_ws not set, use an invalid path to skip it
  for e in ${current_ws:-/dev/null} ~/downstream_ws ~/target_ws ~/base_ws ~/upstream_ws "/opt/ros/$ROS_DISTRO"; do
   if [ -f "$e/setup.bash" ]; then
     set +u
     # shellcheck disable=SC1090
     source "$e/setup.bash"
     set -u
     if [ -n "$*" ]; then
       exec "$@"
     fi
     return 0
   fi
  done
  return 1
)

function ici_with_ws() {
  # shellcheck disable=SC2034
  current_ws=$1; shift
  "$@"
  unset current_ws
}

function _sub_shell() (
  set -u
  eval "$@"
  set +u
)

function ici_hook() {
  local name=${1^^}
  name=${name//[^A-Z0-9_]/_}

  local script=${!name}
  if [ -n "$script" ]; then
    ici_run "$1" _sub_shell "$script"
  fi
}

#######################################
# Starts a timer section on Travis CI
# based on https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/bash/travis_time_start.bash
#      and https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/bash/travis_fold.bash
#
# Globals:
#   DEBUG_BASH (read-only)
#   ICI_FOLD_NAME (write-only)
#   ICI_TIME_ID (write-only)
#   ICI_START_TIME (write-only)
# Arguments:
#   color_wrap (default: 32): Color code for the section delimitter text.
#   exit_code (default: $?): Exit code for display
# Returns:
#   (None)
#######################################

function ici_time_start {
    ici_hook "before_${1}"
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set +x; fi
    ICI_START_TIME=$(date -u +%s%N)
    ICI_TIME_ID="$(printf %08x $((RANDOM * RANDOM)))"
    ICI_FOLD_NAME=$1

    echo # blank line

    if [ "$_DO_NOT_FOLD" != "true" ]; then
        echo -e "ici_fold:start:$ICI_FOLD_NAME"
        echo -en "ici_time:start:$ICI_TIME_ID"
    fi

    ici_color_output $ANSI_BLUE ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    ici_color_output $ANSI_BLUE "Starting function '$ICI_FOLD_NAME'"
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
}

#######################################
# Wraps up the timer section on Travis CI (that's started mostly by ici_time_start function).
# based on https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/bash/travis_time_finish.bash
#
# Globals:
#   DEBUG_BASH (read-only)
#   ICI_FOLD_NAME (from ici_time_start, read-write)
#   ICI_TIME_ID (from ici_time_start, read-only)
#   ICI_START_TIME (from ici_time_start, read-only)
# Arguments:
#   color_wrap (default: 32): Color code for the section delimitter text.
#   exit_code (default: $?): Exit code for display
# Returns:
#   (None)
#######################################
function ici_time_end {
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set +x; fi
    local color_wrap=${1:-${ANSI_GREEN}}
    local exit_code=${2:-$?}
    local name=$ICI_FOLD_NAME

    if [ -z "$ICI_START_TIME" ]; then ici_warn "[ici_time_end] var ICI_START_TIME is not set. You need to call ici_time_start in advance. Returning."; return; fi
    local end_time; end_time=$(date -u +%s%N)
    local elapsed_seconds; elapsed_seconds=$(( (end_time - ICI_START_TIME)/1000000000 ))
    if [ "$_DO_NOT_FOLD" != "true" ]; then
        echo -e "ici_time:end:$ICI_TIME_ID:start=$ICI_START_TIME,finish=$end_time,duration=$((end_time - ICI_START_TIME))"
        echo -en "ici_fold:end:$name"
    fi
    ici_color_output "$color_wrap" "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
    ici_color_output "$color_wrap" "Function '$name' returned with code '${exit_code}' after $(( elapsed_seconds / 60 )) min $(( elapsed_seconds % 60 )) sec"

    unset ICI_FOLD_NAME
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
    ici_hook "after_${name}"

}

function ici_run {
    local name=$1; shift
    ici_time_start "$name"
    "$@"
    ici_time_end
}

#######################################
# exit function with handling for EXPECT_EXIT_CODE, ends the current fold if necessary
#
# Globals:
#   EXPECT_EXIT_CODE (read-only)
#   ICI_FOLD_NAME (from ici_time_start, read-only)
# Arguments:
#   exit_code (default: $?)
# Returns:
#   (None)
#######################################
function ici_exit {
    local exit_code=${1:-$?}  # If 1st arg is not passed, set last error code.
    trap - EXIT # Reset signal handler since the shell is about to exit.

    # end fold if needed
    if [ -n "$ICI_FOLD_NAME" ]; then
        if [ "$exit_code" -ne "0" ]; then color_wrap=${ANSI_RED}; fi  # Red color for errors
        ici_time_end "$color_wrap" "$exit_code"
    fi

    if [ "$exit_code" == "${EXPECT_EXIT_CODE:-0}" ]; then
        exit 0
    elif [ "$exit_code" == "0" ]; then # 0 was not expected
        exit 1
    fi

    exit "$exit_code"
}

function ici_warn {
    ici_color_output ${ANSI_YELLOW} "$*"
}

function ici_mark_deprecated {
  if ! [ "$IN_DOCKER" ]; then
    local e=$1
    shift
    if [ "${!e}" ]; then
      ici_warn "'$e' is deprecated. $*"
    fi
  fi
}

#######################################
# Print an error message and calls "exit"
#
# * Wraps the section that is started by ici_time_start function with the echo color red (${ANSI_RED}).
# * exit_code is taken from second argument or from the previous comman.
# * If the final exit_code is 0, this function will exit 1 instead to enforce a test failure
#
# Globals:
#   (None)
# Arguments:
#   message (optional)
#   exit_code (default: $?)
# Returns:
#   (None)
#######################################
function ici_error {
    local exit_code=${2:-$?} #
    if [ -n "$1" ]; then
        ici_color_output ${ANSI_RED} "$1"
    fi
    if [ "$exit_code" == "0" ]; then # 0 is not error
        ici_exit 1
    fi
    ici_exit "$exit_code"
}

function ici_enforce_deprecated {
    local e=$1
    shift
    if [ "${!e}" ]; then
      ici_error "'$e' is not used anymore. $*"
    fi
}

function ici_retry {
  local tries=$1; shift
  local ret=0

  for ((i=1;i<=tries;i++)); do
    "$@" && return 0
    ret=$?
    sleep 1;
  done

  ici_color_output ${ANSI_RED} "'$*' failed $tries times"
  return $ret
}

function ici_quiet {
    local out; out=$(mktemp)
    # shellcheck disable=SC2216
    "$@" &> "$out" | true # '|| err=$?' disables errexit
    local err=${PIPESTATUS[0]}
    if [ "$err" -ne 0 ]; then
        cat "$out"
    fi
    rm -rf "$out"
    return "$err"
}

function ici_asroot {
  if command -v sudo > /dev/null; then
      sudo "$@"
  else
      "$@"
  fi
}

function ici_split_array {
    # shellcheck disable=SC2034
    IFS=" " read -r -a "$1" <<< "$*"
}

function ici_parse_env_array {
    # shellcheck disable=SC2034
    eval "$1=(${!2})"
}
