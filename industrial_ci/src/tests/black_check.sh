#!/bin/bash

function install_black() {
  ici_quiet ici_install_pkgs_for_command pip3 python3-pip python3-setuptools python3-wheel
  ici_asroot pip3 install black
}

function run_black_check() {
  DOCKER_IMAGE=${DOCKER_IMAGE:-ros:${ROS_DISTRO}-ros-base} ici_require_run_in_docker # this script must be run in docker

  ici_exec_for_command black ici_run install_black install_black
  ici_run run_black_check black --check --color --diff --verbose --exclude "/\..*/" "$TARGET_WORKSPACE"  # Exclude hidden directories.
}
