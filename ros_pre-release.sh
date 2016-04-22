#!/bin/bash

# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Isaac I. Y. Saito
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
## Author: Isaac I. Y. Saito

set -x

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktravis_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktravis_time:start:$TRAVIS_TIME_ID\e[34m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m"
    set -x
}

function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "travis_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\e[0K"
    echo -e "travis_fold:end:$TRAVIS_FOLD_NAME\e[${_COLOR}m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\e[0m"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME took $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

function error {
    travis_time_end 31
    trap - ERR
    exit 1
}

if [ ! "$PRERELEASE_DOWNSTREAM_DEPTH" ]; then export PRERELEASE_DOWNSTREAM_DEPTH="1"; fi
if [ ! "$PRERELEASE_REPONAME" ]; then NAME_REPO=$(echo $TRAVIS_REPO_SLUG | cut -d'/' -f 1); fi
echo "PRERELEASE_REPONAME = ${PRERELEASE_REPONAME}"

travis_time_start setup_docker

sudo usermod -aG docker ubuntu
# ROS Buildfarm for prerelease http://wiki.ros.org/regression_tests#How_do_I_setup_my_system_to_run_a_prerelease.3F
sudo -E sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
# Buildfarm workaround for Python3 http://wiki.ros.org/regression_tests#How_do_I_setup_my_system_to_run_a_prerelease.3F
sudo -E apt-get update && sudo -E apt-get install python3 python3-pip python-ros-buildfarm
sudo python3 -m pip install -U EmPy

travis_time_end  # setup_docker

travis_time_start setup_prerelease_scripts

mkdir -p /tmp/prerelease_job; cd /tmp/prerelease_job; generate_prerelease_script.py https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml $ROS_DISTRO default ubuntu trusty amd64 ${PRERELEASE_REPONAME} --level $PRERELEASE_DOWNSTREAM_DEPTH --output-dir ./

travis_time_end  # setup_prerelease_scripts

travis_time_start run_prerelease

./prerelease.sh;

travis_time_end  # run_prerelease

cd $TRAVIS_BUILD_DIR  # cd back to the repository's home directory with travis
pwd

