#!/bin/bash

function prepare_black_check() {
  if [ -z "${ROS_DISTRO:-}" ]; then
    export DOCKER_IMAGE=${DOCKER_IMAGE:-python:3}
    export ROS_DISTRO=false
  fi
}

function install_black() {
  ici_install_pkgs_for_command pip3 python3-pip python3-setuptools python3-wheel
  ici_asroot pip3 install black
}

function run_black_check() {
  ici_exec_for_command black ici_step install_black install_black
  ici_step run_black_check black --check --color --diff --verbose --exclude "/\..*/" "$TARGET_WORKSPACE"  # Exclude hidden directories.
}
