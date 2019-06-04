#!/bin/bash

# Copyright (c) 2015, Isaac I. Y. Saito
# Copyright (c) 2017, Mathias Luedtke
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
#
## Greatly inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis

# source_tests.sh script runs integration tests for the target ROS packages.
# It is dependent on environment variables that need to be exported in advance
# (As of version 0.4.4 most of them are defined in env.sh).

ici_require_run_in_docker # this script must be run in docker

#Define some verbose env vars
#verbose build
if [ "$VERBOSE_OUTPUT" ] && [ "$VERBOSE_OUTPUT" == true ]; then
    OPT_VI="-vi"
else
    OPT_VI=""
fi
#verbose run tests
if [ "$VERBOSE_TESTS" == false ]; then
    OPT_RUN_V=""
else
    OPT_RUN_V="-v"
fi

ici_time_start init_ici_environment
# Define more env vars
BUILDER=catkin
ROSWS=wstool

ici_time_end  # init_ici_environment

function catkin {
  local path
  path=$(which catkin) || ici_error "catkin not available. Make sure python-catkin-tools is installed. See also https://github.com/ros-industrial/industrial_ci/issues/216"
  local cmd=$1
  shift
  "$path" "$cmd" -w "$CATKIN_WORKSPACE" "$@"
}

ici_time_start setup_apt

ici_asroot apt-get update -qq

# If more DEBs needed during preparation, define ADDITIONAL_DEBS variable where you list the name of DEB(S, delimitted by whitespace)
if [ "$ADDITIONAL_DEBS" ]; then
    add_debs=($ADDITIONAL_DEBS)
    ici_asroot apt-get install -qq -y "${add_debs[@]}" || ici_error "One or more additional deb installation is failed. Exiting."
fi
# shellcheck source=/dev/null
source "/opt/ros/$ROS_DISTRO/setup.bash"

ici_time_end  # setup_apt

if [ "$CCACHE_DIR" ]; then
    ici_time_start setup_ccache
    ici_asroot apt-get install -qq -y ccache || ici_error "Could not install ccache. Exiting."
    export PATH="/usr/lib/ccache:$PATH"
    ici_time_end  # setup_ccache
fi

ici_time_start setup_rosdep

# Setup rosdep
rosdep --version
if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
    ici_asroot rosdep init
fi

update_opts=()
if [ "$ROS_VERSION_EOL" = true ] && rosdep update --help | grep -q -- --include-eol-distros; then
  update_opts+=(--include-eol-distros)
fi

ici_retry 2 rosdep update "${update_opts[@]}"

ici_time_end  # setup_rosdep

ici_time_start setup_rosws

## BEGIN: travis' install: # Use this to install any prerequisites or dependencies necessary to run your build ##
# Create workspace
export CATKIN_WORKSPACE=~/catkin_ws
mkdir -p "$CATKIN_WORKSPACE/src"
if [ ! -f "$CATKIN_WORKSPACE/src/.rosinstall" ]; then
  "$ROSWS" init "$CATKIN_WORKSPACE/src"
fi
case "$UPSTREAM_WORKSPACE" in
debian)
    echo "Obtain deb binary for upstream packages."
    ;;
file) # When UPSTREAM_WORKSPACE is file, the dependended packages that need to be built from source are downloaded based on $ROSINSTALL_FILENAME file.
    # Prioritize $ROSINSTALL_FILENAME.$ROS_DISTRO if it exists over $ROSINSTALL_FILENAME.
    if [ -e "$TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO" ]; then
        # install (maybe unreleased version) dependencies from source for specific ros version
        "$ROSWS" merge -t "$CATKIN_WORKSPACE/src" "file://$TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO"
    elif [ -e "$TARGET_REPO_PATH/$ROSINSTALL_FILENAME" ]; then
        # install (maybe unreleased version) dependencies from source
        "$ROSWS" merge -t "$CATKIN_WORKSPACE/src" "file://$TARGET_REPO_PATH/$ROSINSTALL_FILENAME"
    else
        ici_error "UPSTREAM_WORKSPACE file '$TARGET_REPO_PATH/$ROSINSTALL_FILENAME[.$ROS_DISTRO]' does not exist"
    fi
    ;;
http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
    "$ROSWS" merge -t "$CATKIN_WORKSPACE/src" "$UPSTREAM_WORKSPACE"
    ;;
esac

# download upstream packages into workspace
if [ -e "$CATKIN_WORKSPACE/src/.rosinstall" ]; then
    env
    # ensure that the target is not in .rosinstall
    if (cd "$CATKIN_WORKSPACE/src"; "$ROSWS" rm "$TARGET_REPO_NAME" 2> /dev/null); then
      echo "$ROSWS ignored $TARGET_REPO_NAME found in $CATKIN_WORKSPACE/src/.rosinstall file. Its source fetched from your repository is used instead." # TODO: add warn function
    fi
    "$ROSWS" update -t "$CATKIN_WORKSPACE/src"
fi
# TARGET_REPO_PATH is the path of the downstream repository that we are testing. Link it to the catkin workspace
ln -sf "$TARGET_REPO_PATH" "$CATKIN_WORKSPACE/src"

if [ "${USE_MOCKUP// }" != "" ]; then
    if [ ! -d "$TARGET_REPO_PATH/$USE_MOCKUP" ]; then
        ici_error "mockup directory '$USE_MOCKUP' does not exist"
    fi
    ln -sf "$TARGET_REPO_PATH/$USE_MOCKUP" "$CATKIN_WORKSPACE/src"
fi

catkin config --install
catkin_config=($CATKIN_CONFIG)
if [ ${#catkin_config[@]} -ne 0 ]; then eval catkin config "${catkin_config[@]}"; fi

cmake_args=(${CMAKE_ARGS})
if [ ${#cmake_args[@]} -gt 0 ]; then
  catkin config --cmake-args "${cmake_args[@]}"
fi

ici_time_end  # setup_rosws


# execute BEFORE_SCRIPT in repository, exit on errors
if [ "${BEFORE_SCRIPT// }" != "" ]; then
  ici_time_start before_script

  bash -e -c "cd $TARGET_REPO_PATH; ${BEFORE_SCRIPT}"

  ici_time_end  # before_script
fi

ici_time_start rosdep_install

rosdep_opts=(-q --from-paths $CATKIN_WORKSPACE/src --ignore-src --rosdistro $ROS_DISTRO -y)
if [ -n "$ROSDEP_SKIP_KEYS" ]; then
  rosdep_opts+=(--skip-keys "$ROSDEP_SKIP_KEYS")
fi
set -o pipefail # fail if rosdep install fails
rosdep install "${rosdep_opts[@]}" | { grep "executing command" || true; }
set +o pipefail

ici_time_end  # rosdep_install

if [ "$CATKIN_LINT" == "true" ] || [ "$CATKIN_LINT" == "pedantic" ]; then
    ici_time_start catkin_lint
    ici_asroot pip install catkin-lint

    lint_args=($CATKIN_LINT_ARGS)
    if [ "$CATKIN_LINT" == "pedantic" ]; then
    	lint_args+=(--strict -W2)
    fi
    if catkin_lint --explain "${lint_args[@]}" "$TARGET_REPO_PATH"; then
      echo "catkin_lint passed."
    else
      ici_error "catkin_lint failed by either/both errors and/or warnings"
    fi
    ici_time_end  # catkin_lint
fi

ici_time_start catkin_build

# for catkin
export TARGET_PKGS
export PKGS_DOWNSTREAM

if [ "${TARGET_PKGS// }" == "" ]; then TARGET_PKGS=$(catkin_topological_order "${TARGET_REPO_PATH}" --only-names); fi
# fall-back to all workspace packages if target repo does not contain any packages (#232)
if [ "${TARGET_PKGS// }" == "" ]; then TARGET_PKGS=$(catkin_topological_order "$CATKIN_WORKSPACE/src" --only-names); fi
if [ "${PKGS_DOWNSTREAM// }" == "" ]; then PKGS_DOWNSTREAM=$( [ "${BUILD_PKGS_WHITELIST// }" == "" ] && echo "$TARGET_PKGS" || echo "$BUILD_PKGS_WHITELIST"); fi

pkgs_downstream=($PKGS_DOWNSTREAM)
pkgs_whitelist=($BUILD_PKGS_WHITELIST)

catkin_parallel_jobs=($CATKIN_PARALLEL_JOBS)
ros_parallel_jobs=($ROS_PARALLEL_JOBS)
catkin_parallel_test_jobs=($CATKIN_PARALLEL_TEST_JOBS)
ros_parallel_test_jobs=($ROS_PARALLEL_TEST_JOBS)

if [ "$BUILDER" == catkin ]; then catkin build $OPT_VI --summarize  --no-status "${pkgs_whitelist[@]}" "${catkin_parallel_jobs[@]}" --make-args "${ros_parallel_jobs[@]}" ; fi

ici_time_end  # catkin_build

if [ "$NOT_TEST_BUILD" != "true" ]; then
    ici_time_start catkin_build_downstream_pkgs
    if [ "$BUILDER" == catkin ]; then
        catkin build $OPT_VI --summarize  --no-status "${pkgs_downstream[@]}" "${catkin_parallel_jobs[@]}" --make-args "${ros_parallel_jobs[@]}"
    fi
    ici_time_end  # catkin_build_downstream_pkgs

    ici_time_start catkin_build_tests
    if [ "$BUILDER" == catkin ]; then
        catkin build --no-deps --catkin-make-args tests -- $OPT_VI --summarize  --no-status "${pkgs_downstream[@]}" "${catkin_parallel_jobs[@]}" --make-args "${ros_parallel_jobs[@]}" --
    fi
    ici_time_end  # catkin_build_tests

    ici_time_start catkin_run_tests
    if [ "$BUILDER" == catkin ]; then
        catkin build --no-deps --catkin-make-args run_tests -- $OPT_RUN_V --no-status "${pkgs_downstream[@]}" "${catkin_parallel_test_jobs[@]}" --make-args "${ros_parallel_test_jobs[@]}" --
        if [ "${ROS_DISTRO}" == "hydro" ]; then
            PATH=/usr/local/bin:$PATH  # for installed catkin_test_results
            PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH

            if [ "${ROS_LOG_DIR// }" == "" ]; then export ROS_LOG_DIR=~/.ros/test_results; fi # http://wiki.ros.org/ROS/EnvironmentVariables#ROS_LOG_DIR
            if [ "$BUILDER" == catkin ] && [ -e "$ROS_LOG_DIR" ]; then catkin_test_results --all "$ROS_LOG_DIR" || error; fi
            if [ "$BUILDER" == catkin ] && [ -e "$CATKIN_WORKSPACE/build/" ]; then catkin_test_results --all "$CATKIN_WORKSPACE/build/" || error; fi
            if [ "$BUILDER" == catkin ] && [ -e ~/.ros/test_results/ ]; then catkin_test_results --all ~/.ros/test_results/ || error; fi
        else
            catkin_test_results --verbose "$CATKIN_WORKSPACE" || error
        fi
    fi
    ici_time_end  # catkin_run_tests
fi

if [ "$NOT_TEST_INSTALL" != "true" ]; then

    ici_time_start catkin_install_run_tests

    EXIT_STATUS=0
    # Test if the unit tests in the packages in the downstream repo pass.
    if [ "$BUILDER" == catkin ]; then
      for pkg in $PKGS_DOWNSTREAM; do
        if [ ! -d "$CATKIN_WORKSPACE/install/share/$pkg" ]; then continue; fi # skip meta-packages

        echo "[$pkg] Started testing..."
        rostest_files=$(find "$CATKIN_WORKSPACE/install/share/$pkg" -iname '*.test')
        echo "[$pkg] Found $(echo "$rostest_files" | wc -w) tests."
        for test_file in $rostest_files; do
          echo "[$pkg] Testing $test_file"
          "$CATKIN_WORKSPACE/install/env.sh" rostest "$test_file" || EXIT_STATUS=$?
          if [ $EXIT_STATUS != 0 ]; then
            echo -e "[$pkg] Testing again the failed test: $test_file.\e[${ANSI_RED}m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m"
            "$CATKIN_WORKSPACE/install/env.sh" rostest --text "$test_file"
            echo -e "[$pkg] Testing again the failed test: $test_file.\e[${ANSI_RED}m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\e[0m"
          fi
        done
      done
      [ $EXIT_STATUS -eq 0 ] || ici_error # unless all tests pass, raise error
    fi

    ici_time_end  # catkin_install_run_tests

fi
