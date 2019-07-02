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

# ros_prerelease.sh script runs ROS Pre-release Test.
# It is dependent on environment variables that need to be exported in advance
# (As of version 0.4.4 most of them are defined in env.sh).

function setup_environment() {
    export WORKSPACE
    WORKSPACE=$(mktemp -d)
    echo "WORKSPACE: $WORKSPACE"

    if [ -n "$DOCKER_PORT" ]; then
        DIND_OPTS=(-e "DOCKER_HOST=$DOCKER_PORT")
        user_cmd="useradd ci"
    elif [ -e /var/run/docker.sock ]; then
        DIND_OPTS=(-v /var/run/docker.sock:/var/run/docker.sock)
        user_cmd="groupadd -o -g $(stat -c%g /var/run/docker.sock) host_docker && useradd -G host_docker ci"
    else
        ici_error "Could not detect docker settings"
    fi
    ici_quiet docker build -t "industrial-ci/prerelease" - <<EOF
FROM ros:melodic-ros-core
RUN apt-get update -qq \
    && apt-get -qq install --no-install-recommends -y \
        docker.io \
        git \
        python-ros-buildfarm \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN $user_cmd
USER ci
ENV WORKSPACE $WORKSPACE
WORKDIR $WORKSPACE
ENTRYPOINT ["/opt/ros/melodic/env.sh"]
EOF
}

function run_in_prerelease_docker() {
    ROS_DISTRO='' ici_run_cmd_in_docker "${DIND_OPTS[@]}" \
                                      -v "$WORKSPACE:$WORKSPACE:rw" \
                                      -e TRAVIS \
                                      "industrial-ci/prerelease" \
                                      "$@"

}
function run_ros_prerelease() {
    # Environment vars.
    local downstream_depth=${PRERELEASE_DOWNSTREAM_DEPTH:-"0"}

    ici_time_start setup_environment
    setup_environment
    ici_time_end  # setup_environment

    ici_time_start setup_prerelease_scripts
    mkdir -p "$WORKSPACE/ws/src/"
    local reponame=${PRERELEASE_REPONAME:-$TARGET_REPO_NAME}
    cp -a "$TARGET_REPO_PATH" "$WORKSPACE/ws/src/$reponame"

    # ensure access rights
    ROS_DISTRO='' ici_run_cmd_in_docker "${DIND_OPTS[@]}" -v "$WORKSPACE:$WORKSPACE:rw"  --user root  "industrial-ci/prerelease" chown -R ci:ci "$WORKSPACE"


    if [ "${USE_MOCKUP// }" != "" ]; then
        if [ ! -d "$TARGET_REPO_PATH/$USE_MOCKUP" ]; then
            ici_error "mockup directory '$USE_MOCKUP' does not exist"
        fi
        cp -a "$TARGET_REPO_PATH/$USE_MOCKUP" "$WORKSPACE/ws/src"
    fi

    run_in_prerelease_docker generate_prerelease_script.py https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml "$ROS_DISTRO" default "$OS_NAME" "$OS_CODE_NAME" "${OS_ARCH:-amd64}" --level "$downstream_depth" --output-dir . --custom-repo "$reponame::::"
    ici_time_end  # setup_prerelease_scripts

    ici_time_start prerelease.sh
    run_in_prerelease_docker env ABORT_ON_TEST_FAILURE=1 ./prerelease.sh -y
    ici_time_end  # prerelease.sh

    echo 'ROS Prerelease Test went successful.'
}
