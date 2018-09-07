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

#######################################
# Starts a timer section on Travis CI
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
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set +x; fi
    ICI_START_TIME=$(date +%s%N)
    ICI_TIME_ID=$(printf "%x" $ICI_START_TIME)
    ICI_FOLD_NAME=$1
    if [ "$_DO_NOT_FOLD" != "true" ]; then
        echo -e "\e[0Kici_fold:start:$ICI_FOLD_NAME"
        echo -en "\e[0Kici_time:start:$ICI_TIME_ID"
    fi
    ici_color_output $ANSI_BLUE ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
}

#######################################
# Wraps up the timer section on Travis CI (that's started mostly by ici_time_start function).
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

    if [ -z $ICI_START_TIME ]; then echo '[ici_time_end] var ICI_START_TIME is not set. You need to call `ici_time_start` in advance. Rerutning.'; return; fi
    local end_time=$(date +%s%N)
    local elapsed_seconds=$(( ($end_time - $ICI_START_TIME)/1000000000 ))
    if [ "$_DO_NOT_FOLD" != "true" ]; then
        echo -e "ici_time:end:$ICI_TIME_ID:start=$ICI_START_TIME,finish=$end_time,duration=$(($end_time - $ICI_START_TIME))\e[0K"
        echo -en "ici_fold:end:$ICI_FOLD_NAME"
    fi
    ici_color_output "$color_wrap" "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
    echo -e "\e[0K\e[${color_wrap}mFunction $ICI_FOLD_NAME returned with code '${exit_code}' after $(( $elapsed_seconds / 60 )) min $(( $elapsed_seconds % 60 )) sec \e[0m"

    unset ICI_FOLD_NAME
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
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
        if [ $exit_code -ne "0" ]; then color_wrap=${ANSI_RED}; fi  # Red color for errors
        ici_time_end "$color_wrap" "$exit_code"
    fi

    if [ "$exit_code" == "${EXPECT_EXIT_CODE:-0}" ]; then
        exit 0
    elif [ "$exit_code" == "0" ]; then # 0 was not expected
        exit 1
    fi

    exit $exit_code
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
function error {
    local exit_code=${2:-$?} #
    if [ -n "$1" ]; then
        ici_color_output ${ANSI_RED} "$1"
    fi
    if [ "$exit_code" == "0" ]; then # 0 is not error
        ici_exit 1
    fi
    ici_exit $exit_code
}

function ici_enforce_deprecated {
    local e=$1
    shift
    if [ "${!e}" ]; then
      error "'$e' is not used anymore. $*"
    fi
}

#######################################
# Install and run yamllint.
#
# Globals:
#   (None)
# Arguments:
#    yamllint_conf: config to pass to yamllint (optional)
#    target_paths: paths to pass yamllint (optional)
# Returns:
#   Pass/Fail (bool)
#######################################
function run_yamllint {
    target_path_default="$TARGET_REPO_PATH"
    # If there are '*rosinstall*' files in the repo, add to the search criteria.
    files_rosinstall=$(find . -path ./.git -prune -o -iname "*rosinstall*" -print)
    if [ $? == 0 ]; then target_path_default="$files_rosinstall $target_path_default"; fi

    yamllint_conf=$1
    target_paths=${2:-"$target_path_default"}  # multiple elements delimitted by space.
    sudo apt-get install -qq -y python3-pkg-resources yamllint || (echo "WARN: Required package 'yaml_lint' isn't available. Skipping to check yaml files." && return);

    if [ -z "$yamllint_conf" ]; then
        # Get the yamllint version and assign an appropriate version of config file.
        yamllint_v_output=$(yamllint --version)
        for yamllint_version in $yamllint_v_output; do     echo "yamllint version: ${yamllint_version}"; done

        case $yamllint_version in
            1.2.1)  # Ubuntu Xenial https://packages.ubuntu.com/xenial/yamllint
                yamllint_conf="${ICI_SRC_PATH}"/yamllint_config_v1.2.1.yaml
                ;;
            1.5.0)
                yamllint_conf="${ICI_SRC_PATH}"/yamllint_config_v1.5.0.yaml
                ;;
            *)
                yamllint_conf="${ICI_SRC_PATH}"/yamllint_config_v1.8.2.yaml
                ;;
        esac
    fi

    # Need to remove quotes from target_paths https://stackoverflow.com/a/10943857/577001
    yamllint -c "${yamllint_conf}" $target_paths;
    return $?
}

if ! which sudo > /dev/null; then
  function sudo {
    "$@"
  }
fi
