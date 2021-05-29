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

export _FOLDING_TYPE=${_FOLDING_TYPE:-none}
export ICI_FOLD_NAME=${ICI_FOLD_NAME:-}
export ICI_START_TIME=${ICI_START_TIME:-}
export ICI_TIME_ID=${ICI_TIME_ID:-}

function ici_color_output {
  local c=$1
  shift
  echo -e "\e[${c}m$*\e[0m"
}

function ici_ansi_cleared_line {
  echo -en "$*\r\e[0K"
}

function ici_set_u {
  [[ "${BASH_VERSINFO[0]}_${BASH_VERSINFO[1]}" < "4_4" ]] || set -u
}
function ici_with_unset_variables {
  set +u
  "$@"
  ici_set_u
}

function _sub_shell() (
  function rosenv() {
    # if current_ws not set, use an invalid path to skip it
    for e in ${current_ws:-/dev/null}/install "$BASEDIR/downstream_ws/install" "$BASEDIR/target_ws/install" "$BASEDIR/base_ws/install" "$BASEDIR/upstream_ws/install" "$UNDERLAY"; do
    if [ -f "$e/setup.bash" ]; then
      ici_source_setup "$e"
      if [ -n "$*" ]; then
        (exec "$@")
      fi
      return 0
    fi
    done
    return 1
  }
  eval "$@"
)

function ici_hook() {
  local name=${1^^}
  name=${name//[^A-Z0-9_]/_}
  local name_embed="${name}_EMBED"

  local script=${!name:-}
  local script_embed=${!name_embed:-}

  if [ -n "$script" ] || [ -n "$script_embed" ] ; then
    ici_time_start "$1"

    if [ -n "$script" ]; then
      _sub_shell "$script"
    fi

    if [ -n "$script_embed" ]; then
      eval "$script_embed"
      ici_set_u
    fi

    ici_time_end
  fi
}

#######################################
# Starts a timer section in a folding section
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

    ici_start_fold "$ICI_TIME_ID" "$ICI_FOLD_NAME" "$ICI_START_TIME"
    ici_color_output $ANSI_BLUE "$ICI_FOLD_NAME"
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
}

#######################################
# Wraps up the timer section that was started by ici_time_start function
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

    echo -en "\e[${color_wrap}m"  # just set color, no output
    ici_end_fold "$ICI_TIME_ID" "$name" "$ICI_START_TIME" "$end_time"
    ici_color_output "$color_wrap" "'$name' returned with code '${exit_code}' after $(( elapsed_seconds / 60 )) min $(( elapsed_seconds % 60 )) sec"

    ICI_FOLD_NAME=
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

    local cleanup=()
    # shellcheck disable=SC2016
    IFS=: command eval 'cleanup=(${_CLEANUP})'
    for c in "${cleanup[@]}"; do
      ici_warn Cleaning up "${c/#\~/$HOME}"
      rm -rf "${c/#\~/$HOME}"
    done

    # end fold if needed
    if [ -n "$ICI_FOLD_NAME" ]; then
        local color_wrap=${ANSI_GREEN}
        if [ "$exit_code" -ne "0" ]; then color_wrap=${ANSI_RED}; fi  # Red color for errors
        ici_time_end "$color_wrap" "$exit_code"
    fi

    if [ "$exit_code" == "$EXPECT_EXIT_CODE" ]; then
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
  local e=$1
  shift
  if [ "${!e:-}" ]; then
    ici_warn "'$e' is deprecated. $*"
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
        >&2 ici_color_output ${ANSI_RED} "$1"
    fi
    if [ "$exit_code" == "0" ]; then # 0 is not error
        ici_exit 1
    fi
    ici_exit "$exit_code"
}

function ici_enforce_deprecated {
    local e=$1
    shift
    if [ "${!e:-}" ]; then
      ici_error "'$e' is not used anymore. $*"
    fi
}

function ici_rename_deprecated() {
  local old=$1
  shift
  local new=$1
  shift
  if [ "${!old:-}" ]; then
      local value=${!old}
      ici_warn "'$old' is deprecated. Use '$new=$value' instead"
      export "$new"="$value"
  fi
}

function ici_migrate_hook() {
  local oldname=${1^^}
  oldname=${oldname//[^A-Z0-9_]/_}
  local newname=${2^^}
  newname=${newname//[^A-Z0-9_]/_}

  mapfile -t envs < <(env | grep -oE "(BEFORE_|AFTER_)+$oldname(_EMBED)?")

  for oldhook in "${envs[@]}"; do
    local newhook=${oldhook/$oldname/$newname}
    ici_warn "hook '$oldhook' was renamed to '$newhook'."
    eval "export $newhook=\$$oldhook"
  done
}

function ici_removed_hook() {
  local oldname=${1^^}
  shift
  oldname=${oldname//[^A-Z0-9_]/_}

  mapfile -t envs < <(env | grep -oE "(BEFORE_|AFTER_)+$oldname(_EMBED)?")

  for oldhook in "${envs[@]}"; do
    ici_enforce_deprecated "$oldhook" "$@"
  done
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
    # shellcheck disable=SC2260
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

function ici_exec_for_command {
  local command=$1; shift
  if ! command -v "$command" > /dev/null; then
    "$@"
  fi
}

function ici_split_array {
    # shellcheck disable=SC2034
    IFS=" " read -r -a "$1" <<< "$*"
}

function ici_parse_env_array {
    # shellcheck disable=SC2034
    eval "$1=(${!2:-})"
}

function ici_parse_jobs {
  local -n _ici_parse_jobs_res=$1
  # shellcheck disable=SC2034
  _ici_parse_jobs_res=${!2:-}

  case "$_ici_parse_jobs_res" in
  "")
      _ici_parse_jobs_res="$3";;
  "true")
      _ici_parse_jobs_res="0";;
  "false")
      _ici_parse_jobs_res="1";;
  *)
      if ! [[ "$_ici_parse_jobs_res" =~ ^[0-9]+$ ]]; then
          ici_error "cannot parse $2=$_ici_parse_jobs_res as a number"
      fi
      ;;
  esac
}

function ici_find_nonhidden {
  local path=$1; shift
  local args=()
  if [ $# -gt 0 ]; then
    args=(-a \( "$@" \))
  fi
  find "$path" \( \! \( -path "${path}*/.*" -prune \) \) "${args[@]}"
}

function ici_resolve_component {
  local label=$1
  local group=$2
  for file in "${TARGET_REPO_PATH}/${!label}" "${ICI_SRC_PATH}/$group/${!label}.sh"; do
    if [ -f "$file" ]; then
      echo "$file"
      return
    fi
  done
  ici_error "$label '${!label}' not found"
}

function ici_source_component {
  local script
  script=$(ici_resolve_component "$@")
  # shellcheck disable=SC1090
  source "$script"
}

function ici_check_builder {
  [ -z "$BUILDER" ] || ici_resolve_component BUILDER builders > /dev/null
}

function ici_source_builder {
  ici_source_component BUILDER builders
}

function ici_join_array {
  local sep=$1
  shift
  local res=""
  for elem in "$@"; do
    if [ -n "$elem" ]; then
      res+="$sep$elem"
    fi
  done
  echo "${res#$sep}"
}

function ici_cleanup_later {
  _CLEANUP=$(ici_join_array : "$_CLEANUP" "$@")
}

function ici_make_temp_dir {
  local -n ici_make_temp_dir_res=$1;
  ici_make_temp_dir_res=$(mktemp -d)
  ici_cleanup_later "$ici_make_temp_dir_res"
}

# shellcheck disable=SC1090
ici_source_component _FOLDING_TYPE folding
