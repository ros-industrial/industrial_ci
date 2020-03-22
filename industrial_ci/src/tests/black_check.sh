#!/bin/bash

function run_black_check() {
  DOCKER_IMAGE="$DOCKER_BASE_IMAGE" ici_require_run_in_docker # this script must be run in docker

  ici_time_start install_black
  ici_quiet ici_asroot apt-get install -qq -y python3-pip
  ici_quiet ici_asroot pip3 -q install black
  ici_time_end

  ici_time_start run_black_check
  black --check --diff --verbose --exclude "/\..*/" "$TARGET_WORKSPACE"  # Exclude hidden directories.
  ici_time_end
}
