#!/bin/bash

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

function setup_environment() {
    # ROS Buildfarm for prerelease http://wiki.ros.org/regression_tests#How_do_I_setup_my_system_to_run_a_prerelease.3F
    sudo -E sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo -E apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
    # Buildfarm workaround for Python3 http://wiki.ros.org/regression_tests#How_do_I_setup_my_system_to_run_a_prerelease.3F
    sudo -E apt-get update && sudo -E apt-get -qq install -y python3 python3-pip python-ros-buildfarm ros-${catkin_distro}-catkin
    sudo python3 -m pip install -U EmPy
}

function run_ros_prerelease() {
    # Environment vars.
    local downstream_depth=${PRERELEASE_DOWNSTREAM_DEPTH:-"0"}
    local catkin_distro=$ROS_DISTRO
    local os_code_name

    case "$ROS_DISTRO" in
    "kinetic")
        os_code_name="xenial"
        catkin_distro="jade"
        ;;
    *)
        os_code_name=$(lsb_release -sc)
        ;;
    esac
    os_code_name=${PRERELEASE_OS_CODENAME:-$os_code_name}

    ici_time_start setup_environment
    setup_environment
    ici_time_end  # setup_environment

    ici_time_start setup_prerelease_scripts
    mkdir -p /tmp/prerelease_job; cd /tmp/prerelease_job; 
    local reponame=$PRERELEASE_REPONAME
    if [ ! "$reponame" ]; then
        reponame=$TARGET_REPO_NAME
        mkdir -p catkin_workspace/src
        cp -a $TARGET_REPO_PATH catkin_workspace/src/
    fi
    generate_prerelease_script.py https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml $ROS_DISTRO default ubuntu ${os_code_name} amd64 ${reponame} --level $downstream_depth --output-dir ./
    ici_time_end  # setup_prerelease_scripts

    ici_time_start run_prerelease
    ./prerelease.sh -y;
    ici_time_end  # run_prerelease
    
    ici_time_start show_testresult
    source /opt/ros/${catkin_distro}/setup.bash
    catkin_test_results --verbose || error 'ROS Prerelease Test failed'
    echo 'ROS Prerelease Test went successful.'
    ici_time_end  # show_testresult
    
    cd $TARGET_REPO_PATH  # cd back to the repository's home directory with travis
    pwd
}
