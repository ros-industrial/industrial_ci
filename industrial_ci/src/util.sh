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
ANSI_MAGENTA=35
ANSI_BOLD=1

export _FOLDING_TYPE=${_FOLDING_TYPE:-none}
export TRACE=${TRACE:-false}
export ICI_FOLD_NAME=${ICI_FOLD_NAME:-}
export ICI_START_TIME=${ICI_START_TIME:-}
export ICI_TIME_ID=${ICI_TIME_ID:-}

__ici_log_fd=1
__ici_err_fd=2
__ici_top_level=0

function ici_setup {
    trap 'ici_trap_exit' EXIT # install industrial_ci exit handler
    exec {__ici_log_fd}>&1
    exec {__ici_err_fd}>&2
    __ici_top_level=$BASH_SUBSHELL
}

function ici_redirect {
    1>&"$__ici_log_fd" 2>&"$__ici_err_fd" "$@"
}

function ici_log {
    ici_redirect echo "$@"
}

function ici_color_output {
  local c=$1
  shift
  ici_log -e "\e[${c}m$*\e[0m"
}

function ici_ansi_cleared_line {
  ici_log -en "$*\r\e[0K"
}

function ici_backtrace {
  if [ "$TRACE" = true ]; then
    ici_log
    ici_color_output ${ANSI_MAGENTA} "TRACE:${BASH_SOURCE[2]#$ICI_SRC_PATH/}:${BASH_LINENO[1]} ${FUNCNAME[1]} $*"
    for ((i=3;i<${#BASH_SOURCE[@]};i++)); do
        ici_color_output ${ANSI_MAGENTA} "   AT:${BASH_SOURCE[$i]#$ICI_SRC_PATH/}:${BASH_LINENO[$((i-1))]} ${FUNCNAME[$((i-1))]}"
    done
  fi
}

function ici_trace {
  if [ "$TRACE" = true ]; then
    ici_log
    ici_color_output ${ANSI_MAGENTA} "TRACE:${BASH_SOURCE[2]#$ICI_SRC_PATH/}:${BASH_LINENO[1]} ${FUNCNAME[1]} $*"
  fi
}

function ici_set_u {
  [[ "${BASH_VERSINFO[0]}_${BASH_VERSINFO[1]}" < "4_4" ]] || set -u
}

function ici_with_unset_variables {
  local err=0
  set +u
  "$@" || err=$?
  ici_set_u
  return "$err"
}

function _sub_shell() (
  # shellcheck disable=SC2317
  function rosenv() {
    # if current_ws not set, use an invalid path to skip it
    for e in $(ici_extend_space "${current_ws:-/dev/null}") $(ici_extend_space "$BASEDIR/${PREFIX}downstream_ws") $(ici_extend_space "$BASEDIR/${PREFIX}target_ws") $(ici_extend_space "$BASEDIR/${PREFIX}base_ws") $(ici_extend_space "$BASEDIR/${PREFIX}upstream_ws") "$UNDERLAY"; do
    if [ -f "$e/setup.bash" ]; then
      ici_source_setup "$e"
      if [ -n "$*" ]; then
        (exec "$@") || return
      fi
      return 0
    fi
    done
    return 1
  }
  eval "$*" || ici_exit
)

function _label_hook() {
      ici_log
      # shellcheck disable=SC2001
      ici_color_output ${ANSI_BOLD} "$(sed -e 's/^/$ /' <<< "$1")"
}

function ici_hook() {
  ici_trace "$@"
  local name=${1^^}
  name=${name//[^A-Z0-9_]/_}
  local name_embed="${name}_EMBED"

  local script=${!name:-}
  local script_embed=${!name_embed:-}

  if [ -n "$script" ] || [ -n "$script_embed" ] ; then
    ici_time_start "$1"

    if [ -n "$script" ]; then
      _label_hook "( $script; )"
      _sub_shell "$script" || ici_exit
    fi

    if [ -n "$script_embed" ]; then
      _label_hook "eval \"$script_embed\""
      eval "$script_embed" || ici_exit
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
    ici_hook "before_${1}" || ici_exit
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set +x; fi
    ICI_START_TIME=$(date -u +%s%N)
    ICI_TIME_ID="$(printf %08x $((RANDOM * RANDOM)))"
    ICI_FOLD_NAME=$1

    ici_log # blank line

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

    ici_log -en "\e[${color_wrap}m"  # just set color, no output
    ici_end_fold "$ICI_TIME_ID" "$name" "$ICI_START_TIME" "$end_time"
    ici_color_output "$color_wrap" "'$name' returned with code '${exit_code}' after $(( elapsed_seconds / 60 )) min $(( elapsed_seconds % 60 )) sec"
    ici_report_result "$ICI_FOLD_NAME" "${exit_code}"

    ICI_FOLD_NAME=
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
    ici_hook "after_${name}" || ici_exit
}

function ici_step {
    local name=$1; shift
    ici_time_start "$name"
    "$@" || ici_exit
    ici_time_end
}

function ici_teardown {
    if [  "$BASH_SUBSHELL" -le "$__ici_top_level" ]; then
        local exit_code=$1
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
        elif [ -n "${ICI_RESULT_NAME:-}" ]; then
            ici_report_result "$ICI_RESULT_NAME" "${exit_code}"
        fi

        exec {__ici_log_fd}>&-
        exec {__ici_err_fd}>&-
    fi
}

function ici_trap_exit {
    local exit_code=${1:-$?}

    ici_warn "industrial_ci terminated unexpectedly with exit code '$exit_code'"
    TRACE=true ici_backtrace "$@"
    exit_code=143
    ici_teardown "$exit_code"
    exit "$exit_code"
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
    local exit_code=${1:-$?}
    ici_backtrace "$@"

    ici_teardown "$exit_code"

    if [ "$exit_code" == "$EXPECT_EXIT_CODE" ] ; then
        exit_code=0
    elif [ "$exit_code" == "0" ]; then # 0 was not expected
        exit_code=1
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
        __ici_log_fd=$__ici_err_fd ici_color_output ${ANSI_RED} "$1"
    fi
    if [ "$exit_code" == "0" ]; then # 0 is not error
        exit_code=1
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
  ici_trace "$@"
  local tries=$1; shift
  local ret=0

  for ((i=1;i<=tries;i++)); do
    "$@" && return 0
    ret=$?
    sleep 1;
  done

  ici_color_output ${ANSI_RED} "'$*' failed $tries times"
  return "$ret"
}

function ici_get_log_cmd {
    local post=""
    while true; do
        case "$1" in
            ici_asroot)
                echo -n "sudo "
                ;;
            ici_exec_in_workspace)
                echo -n "( source $2/setup.bash && "
                if [ "$3" != '.' ]; then
                  echo -n "cd $3 && "
                fi
                shift 2
                post="$post; )"
                ;;
            ici_filter)
                post=" | grep -E '$2' "
                shift 1
                ;;
            ici_quiet)
                post=" > /dev/null "
                ;;
            ici_cmd|ici_guard|ici_label)
                ;;
            *)
              echo "$*$post"
              return
        esac
        shift
    done
}

function ici_quiet {
    local out; out=$(mktemp)
    local err=0
    "$@" &> "$out" || err=$?
    if [ "$err" -ne 0 ]; then
        ici_redirect cat "$out"
        rm -rf "$out"
    fi
    rm -rf "$out"
    return "$err"
}

function ici_filter {
    local filter=$1; shift
    local out; out=$(mktemp)
    "$@" | grep -E "$filter" | ici_redirect cat || true
    local err=${PIPESTATUS[0]}
    if [ "$err" -ne 0 ]; then
        ici_redirect cat "$out"
    fi
    rm -rf "$out"
    return "$err"
}


function _ici_guard {
    local err=0
    "$@" || err=$?
    if [ "$err" -ne 0 ]; then
        ici_error "'$(ici_get_log_cmd "$@")' returned with $err" "$err"
    fi
}

function ici_guard {
    ici_trace "$@"
    _ici_guard "$@"
}

function ici_label {
    local cmd; cmd=$(ici_get_log_cmd "$@")
    ici_log
    ici_color_output ${ANSI_BOLD} "$ $cmd"
     "$@"
}

function ici_cmd {
     _ici_guard ici_label "$@"
}

function ici_asroot {
  if [ "$EUID" -ne 0 ] && command -v sudo > /dev/null; then
      sudo "$@"
  else
      "$@"
  fi
}

function ici_exec_for_command {
  ici_trace "$@"
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
  ici_trace "$@"
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
  ici_guard source "$script"
}

function ici_check_builder {
  ici_trace "$@"
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
  echo "${res#"$sep"}"
}

function ici_cleanup_later {
  ici_trace "$@"
  _CLEANUP=$(ici_join_array : "$_CLEANUP" "$@")
}

function ici_make_temp_dir {
  ici_trace "$@"
  local -n ici_make_temp_dir_res=$1;
  ici_make_temp_dir_res=$(mktemp -d)
  ici_log "ici_make_temp_dir: $1 -> $ici_make_temp_dir_res"
  ici_cleanup_later "$ici_make_temp_dir_res"
}

ici_source_component _FOLDING_TYPE folding
