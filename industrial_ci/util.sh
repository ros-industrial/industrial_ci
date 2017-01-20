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

#######################################
# Starts a timer section on Travis CI
#
# Globals:
#   DEBUG_BASH (read-only)
#   TRAVIS_FOLD_NAME (write-only)
#   TRAVIS_TIME_ID (write-only)
#   TRAVIS_START_TIME (write-only)
# Arguments:
#   color_wrap (default: 32): Color code for the section delimitter text.
#   exit_code (default: $?): Exit code for display
# Returns:
#   (None)
#######################################

function ici_time_start {
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set +x; fi
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Kici_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Kici_time:start:$TRAVIS_TIME_ID\e[34m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m"
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
}

#######################################
# Wraps up the timer section on Travis CI (that's started mostly by ici_time_start function).
#
# Globals:
#   DEBUG_BASH (read-only)
#   TRAVIS_FOLD_NAME (from ici_time_start, read-write)
#   TRAVIS_TIME_ID (from ici_time_start, read-only)
#   TRAVIS_START_TIME (from ici_time_start, read-only)
# Arguments:
#   color_wrap (default: 32): Color code for the section delimitter text.
#   exit_code (default: $?): Exit code for display
# Returns:
#   (None)
#######################################
function ici_time_end {
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set +x; fi
    local color_wrap=${1:-32}
    local exit_code=${2:-$?}

    if [ -z $TRAVIS_START_TIME ]; then echo '[ici_time_end] var TRAVIS_START_TIME is not set. You need to call `ici_time_start` in advance. Rerutning.'; return; fi
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "ici_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\e[0K"
    echo -e "ici_fold:end:$TRAVIS_FOLD_NAME\e[${color_wrap}m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\e[0m"
    echo -e "\e[0K\e[${color_wrap}mFunction $TRAVIS_FOLD_NAME returned with code '${exit_code}' after $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec \e[0m"

    unset TRAVIS_FOLD_NAME
    if [ "$DEBUG_BASH" ] && [ "$DEBUG_BASH" == true ]; then set -x; fi
}

#######################################
# exit function with handling for EXPECT_EXIT_CODE, ends the current fold if necessary
#
# Globals:
#   EXPECT_EXIT_CODE (read-only)
#   TRAVIS_FOLD_NAME (from ici_time_start, read-only)
# Arguments:
#   exit_code (default: $?)
# Returns:
#   (None)
#######################################
function ici_exit {
    local exit_code=${1:-$?}  # If 1st arg is not passed, set last error code.
    trap - EXIT # Reset signal handler since the shell is about to exit.

    # end fold if needed
    if [ -n "$TRAVIS_FOLD_NAME" ]; then
        if [ $exit_code -ne "0" ]; then color_wrap=31; fi  # Red color for errors
        ici_time_end "$color_wrap" "$exit_code"
    fi

    if [ "$exit_code" == "${EXPECT_EXIT_CODE:-0}" ]; then
        exit 0
    elif [ "$exit_code" == "0" ]; then # 0 was not expected
        exit 1
    fi

    exit $exit_code
}

#######################################
# Print an error message and calls "exit"
#
# * Wraps the section that is started by ici_time_start function with the echo color red (31).
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
    if [ -n $1 ]; then
        echo "\e[31m$1\e0m" # print error in red
    fi
    if [ "$exit_code" == "0" ]; then # 0 is not error
        exit 1
    fi
    ici_exit $exit_code
}
