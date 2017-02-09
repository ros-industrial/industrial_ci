#!/bin/bash

# Copyright (c) 2016, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Lüdtke
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
    docker build -t "industrial-ci/prerelease" - <<EOF > /dev/null
FROM ubuntu:xenial

RUN apt-get update -qq && apt-get -qq install --no-install-recommends -y wget ca-certificates

RUN echo "deb ${ROS_REPOSITORY_PATH} xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-key adv --keyserver "${APTKEY_STORE_SKS}" --recv-key "${HASHKEY_SKS}" \
    || { wget "${APTKEY_STORE_HTTPS}" -O - | sudo apt-key add -; }

RUN apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y \
        docker.io \
        git \
        python-ros-buildfarm \
        ros-kinetic-catkin \
        sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN groupadd -g $(stat -c%g /var/run/docker.sock) host_docker \
    && groupadd -g $(id -g) buildfarm \
    && useradd -u $(id -u) -g buildfarm -G host_docker buildfarm
USER buildfarm
ENV WORKSPACE $WORKSPACE
WORKDIR $WORKSPACE
ENTRYPOINT ["/opt/ros/kinetic/env.sh"]
EOF
}

function run_in_prerelease_docker() {
    ici_run_cmd_in_docker -v /var/run/docker.sock:/var/run/docker.sock \
                          -v "$WORKSPACE:$WORKSPACE:rw" \
                          "industrial-ci/prerelease" \
                          "$@"

}
function run_ros_prerelease() {
    # Environment vars.
    local downstream_depth=${PRERELEASE_DOWNSTREAM_DEPTH:-"0"}
    export WORKSPACE
    WORKSPACE=$(mktemp -d)
    echo "WORKSPACE: $WORKSPACE:"

    ici_time_start setup_environment
    setup_environment
    ici_time_end  # setup_environment

    ici_time_start setup_prerelease_scripts
    mkdir -p "$WORKSPACE/catkin_workspace/src/"
    local reponame=$PRERELEASE_REPONAME
    local clone_underlay=true
    if [ -z "$reponame" ]; then
        reponame=$TARGET_REPO_NAME
        mkdir -p catkin_workspace/src
        cp -a "$TARGET_REPO_PATH"/* "$WORKSPACE/catkin_workspace/src/"
        clone_underlay=false
    fi
    run_in_prerelease_docker generate_prerelease_script.py https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml "$ROS_DISTRO" default ubuntu "$UBUNTU_OS_CODE_NAME" amd64 "${reponame}" --level "$downstream_depth" --output-dir .
    ici_time_end  # setup_prerelease_scripts

    if [ "$clone_underlay" = true ]; then 
        ici_time_start prerelease_clone_underlay.sh
        run_in_prerelease_docker ./prerelease_clone_underlay.sh
        ici_time_end  # prerelease_clone_underlay
    fi

    ici_time_start prerelease_build_underlay.sh
    run_in_prerelease_docker ./prerelease_build_underlay.sh
    ici_time_end  # prerelease_build_underlay.sh

    if [ "$downstream_depth" != "0" ]; then
        ici_time_start run_prerelease_clone_overlay.sh
        run_in_prerelease_docker ./prerelease_clone_overlay.sh
        ici_time_end  # run_prerelease_clone_overlay
        ici_time_start prerelease_build_overlay.sh
        run_in_prerelease_docker ./prerelease_build_overlay.sh
        ici_time_end  # prerelease_build_overlay.sh
    fi

    echo 'ROS Prerelease Test went successful.'
}
