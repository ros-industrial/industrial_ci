#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
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

ici_enforce_deprecated BEFORE_SCRIPT "Please migrate to new hook system."
ici_enforce_deprecated CATKIN_CONFIG "Explicit catkin configuration is not available anymore."
ici_enforce_deprecated INJECT_QEMU "Please check https://github.com/ros-industrial/industrial_ci/blob/master/doc/migration_guide.md#inject_qemu"
ici_enforce_deprecated DOCKER_FILE "Please build image separately"

if [ -n "${NOT_TEST_INSTALL:-}" ]; then
    if [ "$NOT_TEST_INSTALL" != true ]; then
        ici_enforce_deprecated NOT_TEST_INSTALL "testing installed test files has been removed."
    else
        ici_mark_deprecated NOT_TEST_INSTALL "testing installed test files has been removed, NOT_TEST_INSTALL=false is superfluous"
    fi
fi

if [ -n "${DOCKER_BASE_IMAGE:-}" ]; then
    ici_mark_deprecated DOCKER_BASE_IMAGE "Please set DOCKER_IMAGE=$DOCKER_BASE_IMAGE directly"
    export DOCKER_IMAGE=$DOCKER_BASE_IMAGE
fi

for v in BUILD_PKGS_WHITELIST PKGS_DOWNSTREAM TARGET_PKGS USE_MOCKUP; do
    ici_enforce_deprecated "$v" "Please migrate to new workspace definition"
done

for v in CATKIN_PARALLEL_JOBS CATKIN_PARALLEL_TEST_JOBS ROS_PARALLEL_JOBS ROS_PARALLEL_TEST_JOBS; do
    ici_mark_deprecated "$v" "Please migrate to PARALLEL_BUILDS and/or PARALLEL_TESTS"
done

ici_mark_deprecated ROSINSTALL_FILENAME "Please migrate to new UPSTREAM_WORKSPACE format"
ici_mark_deprecated UBUNTU_OS_CODE_NAME "Was renamed to OS_CODE_NAME."
ici_mark_deprecated DEFAULT_DOCKER_IMAGE "Official ROS Docker images are not the default anymore"

if [ -n "${USE_MOCKUP:-}" ]; then
  if [ -z "$TARGET_WORKSPACE" ]; then
    export TARGET_WORKSPACE="$USE_MOCKUP"
    ici_warn "Replacing 'USE_MOCKUP=$USE_MOCKUP' with 'TARGET_WORKSPACE=$TARGET_WORKSPACE'"
  else
    ici_error "USE_MOCKUP is not supported anymore, please migrate to 'TARGET_WORKSPACE=$TARGET_WORKSPACE $USE_MOCKUP'"
  fi
fi

# legacy support for UPSTREAM_WORKSPACE and USE_DEB
if [ "${UPSTREAM_WORKSPACE:-}" = "debian" ]; then
  ici_warn "Setting 'UPSTREAM_WORKSPACE=debian' is superfluous and gets removed"
  unset UPSTREAM_WORKSPACE
fi

if [ "${USE_DEB:-}" = true ]; then
  if [ "${UPSTREAM_WORKSPACE:-debian}" != "debian" ]; then
    ici_error "USE_DEB and UPSTREAM_WORKSPACE are in conflict"
  fi
  ici_warn "Setting 'USE_DEB=true' is superfluous"
fi

if [ "${UPSTREAM_WORKSPACE:-}" = "file" ] || [ "${USE_DEB:-true}" != true ]; then
  ROSINSTALL_FILENAME="${ROSINSTALL_FILENAME:-.travis.rosinstall}"
  if [ -f  "$TARGET_REPO_PATH/$ROSINSTALL_FILENAME.${ROS_DISTRO:?ROS_DISTRO not set}" ]; then
    ROSINSTALL_FILENAME="$ROSINSTALL_FILENAME.$ROS_DISTRO"
  fi

  if [ "${USE_DEB:-true}" != true ]; then # means UPSTREAM_WORKSPACE=file
      if [ "${UPSTREAM_WORKSPACE:-file}" != "file" ]; then
        ici_error "USE_DEB and UPSTREAM_WORKSPACE are in conflict"
      fi
      ici_warn "Replacing 'USE_DEB=false' with 'UPSTREAM_WORKSPACE=$ROSINSTALL_FILENAME'"
  else
      ici_warn "Replacing 'UPSTREAM_WORKSPACE=file' with 'UPSTREAM_WORKSPACE=$ROSINSTALL_FILENAME'"
  fi
  export UPSTREAM_WORKSPACE="$ROSINSTALL_FILENAME"
fi

if [ "${DOCKER_PULL:-true}" = true ]; then
  ici_migrate_hook prepare_docker_image pull_docker_image
else
  ici_removed_hook prepare_docker_image "Hook 'prepare_docker_image' got removed."
fi

ici_mark_deprecated DOCKER_COMMIT_CREDENTIALS "Credentials will be copied, but never committed!"

ici_rename_deprecated HASHKEY_SKS ROS_REPOSITORY_KEY
ici_rename_deprecated APTKEY_STORE_HTTPS ROS_REPOSITORY_KEY
