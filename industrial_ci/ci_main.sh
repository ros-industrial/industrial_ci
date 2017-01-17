#!/bin/bash

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Isaac I. Y. Saito
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#       * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#       * Neither the name of the Isaac I. Y. Saito, nor the names
#       of its contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
## Greatly inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis
## Author: Isaac I. Y. Saito

## This is a "common" script that can be run on travis CI at a downstream github repository.
## See ./README.rst for the detailed usage.

set -e # exit script on errors
if [ "$DEBUG_BASH" ]; then set -x; fi # print trace if DEBUG

#Define some verbose env vars
if [ "$VERBOSE_OUTPUT" ] && [ "$VERBOSE_OUTPUT" == true ]; then
    export OPT_VI="-vi"
else
    export OPT_VI=""
fi

# Define some env vars that need to come earlier than util.sh
export ICI_PKG_PATH=$(pwd)  # The path on CI service (e.g. Travis CI) of industrial_ci top dir.
export CI_MAIN_PKG=industrial_ci
export HIT_ENDOFSCRIPT=false

source ${ICI_PKG_PATH}/util.sh

trap error EXIT
trap success SIGTERM  # So that this script won't terminate without verifying that all necessary steps are done.

# Start prerelease, and once it finishs then finish this script too.
if [ "$PRERELEASE" == true ]; then
  source ${ICI_PKG_PATH}/ros_pre-release.sh
  run_ros_prerelease
else
  source ${ICI_PKG_PATH}/source_tests.sh
fi

ici_time_start after_script

  cd $TARGET_REPO_PATH
  if [ "${AFTER_SCRIPT// }" != "" ]; then sh -e -c "${AFTER_SCRIPT}"; fi

ici_time_end  # after_script

cd $TARGET_REPO_PATH  # cd back to the repository's home directory with travis

ici_exit 0
