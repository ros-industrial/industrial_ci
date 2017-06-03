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

ici_require_run_in_docker # this script must be run in docker

#Define some verbose env vars
if [ "$VERBOSE_OUTPUT" ] && [ "$VERBOSE_OUTPUT" == true ]; then
    OPT_VI="-vi"
else
    OPT_VI=""
fi

ici_time_start init_ici_environment
# Define more env vars
BUILDER=catkin
ROSWS=wstool

ici_time_end  # init_ici_environment

function catkin {
  local cmd=$1
  shift
  /usr/bin/catkin "$cmd" -w "$CATKIN_WORKSPACE" "$@"
}

ici_time_start setup_ros

sudo apt-get update -qq

# If more DEBs needed during preparation, define ADDITIONAL_DEBS variable where you list the name of DEB(S, delimitted by whitespace)
if [ "$ADDITIONAL_DEBS" ]; then
    sudo apt-get install -q -qq -y $ADDITIONAL_DEBS || error "One or more additional deb installation is failed. Exiting."
fi
source /opt/ros/$ROS_DISTRO/setup.bash

ici_time_end  # setup_ros

ici_time_start setup_rosdep

# Setup rosdep
rosdep --version
if ! [ -d /etc/ros/rosdep/sources.list.d ]; then
    sudo rosdep init
fi
ret_rosdep=1
rosdep update || while [ $ret_rosdep != 0 ]; do sleep 1; rosdep update && ret_rosdep=0 || echo "rosdep update failed"; done

ici_time_end  # setup_rosdep

ici_time_start setup_rosws

## BEGIN: travis' install: # Use this to install any prerequisites or dependencies necessary to run your build ##
# Create workspace
export CATKIN_WORKSPACE=~/catkin_ws
mkdir -p $CATKIN_WORKSPACE/src
$ROSWS init $CATKIN_WORKSPACE/src
case "$UPSTREAM_WORKSPACE" in
debian)
    echo "Obtain deb binary for upstream packages."
    ;;
file) # When UPSTREAM_WORKSPACE is file, the dependended packages that need to be built from source are downloaded based on $ROSINSTALL_FILENAME file.
    # Prioritize $ROSINSTALL_FILENAME.$ROS_DISTRO if it exists over $ROSINSTALL_FILENAME.
    if [ -e $TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO ]; then
        # install (maybe unreleased version) dependencies from source for specific ros version
        $ROSWS merge -t $CATKIN_WORKSPACE/src file://$TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO
    elif [ -e $TARGET_REPO_PATH/$ROSINSTALL_FILENAME ]; then
        # install (maybe unreleased version) dependencies from source
        $ROSWS merge -t $CATKIN_WORKSPACE/src file://$TARGET_REPO_PATH/$ROSINSTALL_FILENAME
    else
        error "UPSTREAM_WORKSPACE file '$TARGET_REPO_PATH/$ROSINSTALL_FILENAME[.$ROS_DISTRO]' does not exist"
    fi
    ;;
http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
    $ROSWS merge -t $CATKIN_WORKSPACE/src $UPSTREAM_WORKSPACE
    ;;
esac

# download upstream packages into workspace
if [ -e $CATKIN_WORKSPACE/src/.rosinstall ]; then
    # ensure that the target is not in .rosinstall
    (cd $CATKIN_WORKSPACE/src; $ROSWS rm $TARGET_REPO_NAME 2> /dev/null \
     && echo "$ROSWS ignored $TARGET_REPO_NAME found in $CATKIN_WORKSPACE/src/.rosinstall file. Its source fetched from your repository is used instead." || true) # TODO: add warn function
    $ROSWS update -t $CATKIN_WORKSPACE/src
fi
# TARGET_REPO_PATH is the path of the downstream repository that we are testing. Link it to the catkin workspace
ln -s $TARGET_REPO_PATH $CATKIN_WORKSPACE/src

if [ "${USE_MOCKUP// }" != "" ]; then
    if [ ! -d "$TARGET_REPO_PATH/$USE_MOCKUP" ]; then
        error "mockup directory '$USE_MOCKUP' does not exist"
    fi
    ln -s "$TARGET_REPO_PATH/$USE_MOCKUP" $CATKIN_WORKSPACE/src
fi

catkin config --install
if [ -n "$CATKIN_CONFIG" ]; then catkin config $CATKIN_CONFIG; fi

ici_time_end  # setup_rosws


# execute BEFORE_SCRIPT in repository, exit on errors
if [ "${BEFORE_SCRIPT// }" != "" ]; then
  ici_time_start before_script

  bash -e -c "cd $TARGET_REPO_PATH; ${BEFORE_SCRIPT}"

  ici_time_end  # before_script
fi

ici_time_start rosdep_install

if [ "${ROSDEP_INSTALL_OPTS// }" == "" ]; then
  ROSDEP_INSTALL_OPTS="-q --ignore-src"
fi
set -o pipefail # fail if rosdep install fails
rosdep install --from-paths $CATKIN_WORKSPACE --rosdistro $ROS_DISTRO -y $ROSDEP_INSTALL_OPTS | { grep "executing command" || true; }
set +o pipefail

ici_time_end  # rosdep_install

ici_time_start catkin_build

# for catkin
if [ "${TARGET_PKGS// }" == "" ]; then export TARGET_PKGS=`catkin_topological_order ${TARGET_REPO_PATH} --only-names`; fi
if [ "${PKGS_DOWNSTREAM// }" == "" ]; then export PKGS_DOWNSTREAM=$( [ "${BUILD_PKGS_WHITELIST// }" == "" ] && echo "$TARGET_PKGS" || echo "$BUILD_PKGS_WHITELIST"); fi
if [ "$BUILDER" == catkin ]; then catkin build $OPT_VI --summarize  --no-status $BUILD_PKGS_WHITELIST $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS            ; fi

ici_time_end  # catkin_build

if [ "$NOT_TEST_BUILD" != "true" ]; then
    ici_time_start catkin_run_tests

    if [ "$BUILDER" == catkin ]; then
        catkin run_tests $OPT_VI --no-deps --no-status $PKGS_DOWNSTREAM $CATKIN_PARALLEL_TEST_JOBS --make-args $ROS_PARALLEL_TEST_JOBS --
        catkin_test_results $CATKIN_WORKSPACE || error
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
        echo "[$pkg] Found $(echo $rostest_files | wc -w) tests."
        for test_file in $rostest_files; do
          echo "[$pkg] Testing $test_file"
          $CATKIN_WORKSPACE/install/env.sh rostest $test_file || EXIT_STATUS=$?
          if [ $EXIT_STATUS != 0 ]; then
            echo -e "[$pkg] Testing again the failed test: $test_file.\e[31m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m"
            $CATKIN_WORKSPACE/install/env.sh rostest --text $test_file
            echo -e "[$pkg] Testing again the failed test: $test_file.\e[31m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\e[0m"
          fi
        done
      done
      [ $EXIT_STATUS -eq 0 ] || error  # unless all tests pass, raise error
    fi

    ici_time_end  # catkin_install_run_tests

fi

ici_time_start test_results

if [ "${ROS_DISTRO}" == "hydro" ]; then
    PATH=/usr/local/bin:$PATH  # for installed catkin_test_results
    PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH

    if [ "${ROS_LOG_DIR// }" == "" ]; then export ROS_LOG_DIR=~/.ros/test_results; fi # http://wiki.ros.org/ROS/EnvironmentVariables#ROS_LOG_DIR
    if [ "$BUILDER" == catkin -a -e $ROS_LOG_DIR ]; then catkin_test_results --all $ROS_LOG_DIR || error; fi
    if [ "$BUILDER" == catkin -a -e $CATKIN_WORKSPACE/build/ ]; then catkin_test_results --all $CATKIN_WORKSPACE/build/ || error; fi
    if [ "$BUILDER" == catkin -a -e ~/.ros/test_results/ ]; then catkin_test_results --all ~/.ros/test_results/ || error; fi
else
    catkin_test_results --verbose $CATKIN_WORKSPACE
fi

ici_time_end  # test_results
