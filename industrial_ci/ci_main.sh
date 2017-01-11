#!/bin/bash

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Isaac I. Y. Saito
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
## Greatly inspired by JSK travis https://github.com/jsk-ros-pkg/jsk_travis
## Author: Isaac I. Y. Saito

## This is a "common" script that can be run on travis CI at a downstream github repository.
## See ./README.rst for the detailed usage.

set -e # exit script on errors
set -x # print trace

# Define some env vars that need to come earlier than util.sh
export ICI_PKG_PATH=$(pwd)  # The path on CI service (e.g. Travis CI) of industrial_ci top dir.
export CI_MAIN_PKG=industrial_ci
export HIT_ENDOFSCRIPT=false

source ${ICI_PKG_PATH}/util.sh

trap error ERR
trap success SIGTERM  # So that this script won't terminate without verifying that all necessary steps are done.

# Start prerelease, and once it finishs then finish this script too.
travis_time_start prerelease_from_travis_sh
if [ "$PRERELEASE" == true ] && [ -e ${ICI_PKG_PATH}/ros_pre-release.sh ]; then
  source ${ICI_PKG_PATH}/ros_pre-release.sh && run_ros_prerelease
  retval_prerelease=$?
  if [ $retval_prerelease -eq 0 ]; then HIT_ENDOFSCRIPT=true; success 0; else error; fi  # Internally called travis_time_end for prerelease_from_travis_sh
  # With Prerelease option, we want to stop here without running the rest of the code.
fi

# Building in 16.04 requires running this script in a docker container
# The Dockerfile in this repository defines a Ubuntu 16.04 container
if [[ "$ROS_DISTRO" == "kinetic" ]] && ! [ "$IN_DOCKER" ]; then
  travis_time_start build_docker_image
  docker build -t industrial-ci/xenial .
  travis_time_end  # build_docker_image

  travis_time_start run_travissh_docker

  #forward ssh agent into docker container
  if [ "$SSH_AUTH_SOCK" ]; then
      export SSH_DOCKER_CMD="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
  else
      export SSH_DOCKER_CMD=""
  fi

  docker_target_repo_path=/root/ci_src
  docker_ici_pkg_path=${ICI_PKG_PATH/$TARGET_REPO_PATH/$docker_target_repo_path}
  docker run \
      --env-file ${ICI_PKG_PATH}/docker.env \
      -e TARGET_REPO_PATH=$docker_target_repo_path \
      $SSH_DOCKER_CMD \
      -v $TARGET_REPO_PATH/:$docker_target_repo_path industrial-ci/xenial \
      /bin/bash -c "cd $docker_ici_pkg_path; source ./ci_main.sh;"
  retval=$?
  if [ $retval -eq 0 ]; then HIT_ENDOFSCRIPT=true; success 0; else exit; fi  # Call  travis_time_end  run_travissh_docker
fi

travis_time_start init_travis_environment
#setup github.com ssh key
mkdir -p ~/.ssh
ssh-keyscan github.com >> ~/.ssh/known_hosts 2> /dev/null
# Define more env vars
BUILDER=catkin
ROSWS=wstool
# For compatibilility with hydro catkin, which has no --verbose flag
CATKIN_TEST_RESULTS_CMD="catkin_test_results"
if [ catkin_test_results --verbose 1>/dev/null 2>/dev/null; then CATKIN_TEST_RESULTS_CMD="catkin_test_results --verbose"; fi

if [ ! "$CATKIN_PARALLEL_JOBS" ]; then export CATKIN_PARALLEL_JOBS="-p4"; fi
if [ ! "$CATKIN_PARALLEL_TEST_JOBS" ]; then export CATKIN_PARALLEL_TEST_JOBS="$CATKIN_PARALLEL_JOBS"; fi
if [ ! "$ROS_PARALLEL_JOBS" ]; then export ROS_PARALLEL_JOBS="-j8"; fi
if [ ! "$ROS_PARALLEL_TEST_JOBS" ]; then export ROS_PARALLEL_TEST_JOBS="$ROS_PARALLEL_JOBS"; fi
# If not specified, use ROS Shadow repository http://wiki.ros.org/ShadowRepository
if [ ! "$ROS_REPOSITORY_PATH" ]; then export ROS_REPOSITORY_PATH="http://packages.ros.org/ros-shadow-fixed/ubuntu"; fi
# .rosintall file name 
if [ ! "$ROSINSTALL_FILENAME" ]; then export ROSINSTALL_FILENAME=".travis.rosinstall"; fi 
# For apt key stores
if [ ! "$APTKEY_STORE_HTTPS" ]; then export APTKEY_STORE_HTTPS="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"; fi
if [ ! "$APTKEY_STORE_SKS" ]; then export APTKEY_STORE_SKS="hkp://ha.pool.sks-keyservers.net"; fi  # Export a variable for SKS URL for break-testing purpose.
if [ ! "$HASHKEY_SKS" ]; then export HASHKEY_SKS="0xB01FA116"; fi
if [ "$USE_DEB" ]; then  # USE_DEB is deprecated. See https://github.com/ros-industrial/industrial_ci/pull/47#discussion_r64882878 for the discussion.
    if [ "$USE_DEB" != "true" ]; then export UPSTREAM_WORKSPACE="file";
    else export UPSTREAM_WORKSPACE="debian";
    fi
fi
if [ ! "$UPSTREAM_WORKSPACE" ]; then export UPSTREAM_WORKSPACE="debian"; fi

git branch --all
if [ "`git diff origin/master FETCH_HEAD $ICI_PKG_PATH`" != "" ] ; then DIFF=`git diff origin/master FETCH_HEAD $ICI_PKG_PATH | grep .*Subproject | sed s'@.*Subproject commit @@' | sed 'N;s/\n/.../'`; (cd $CI_MAIN_PKG/;git log --oneline --graph --left-right --first-parent --decorate $DIFF) | tee /tmp/$$-travis-diff.log; grep -c '<' /tmp/$$-travis-diff.log && exit 1; echo "ok"; fi

travis_time_end  # init_travis_environment

travis_time_start setup_ros

echo "Testing branch $TRAVIS_BRANCH of $TARGET_REPO_NAME"  # $TARGET_REPO_NAME is the repo where this job is triggered from, and the variable is expected to be passed externally (industrial_ci/travis.sh should be taking care of it)
# Set apt repo
sudo -E sh -c 'echo "deb $ROS_REPOSITORY_PATH `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
# Common ROS install preparation
# apt key acquisition. Since keyserver may often become accessible, backup method is added.
sudo apt-key adv --keyserver $APTKEY_STORE_SKS --recv-key $HASHKEY_SKS || ((echo 'Fetching apt key from SKS keyserver somehow failed. Trying to get one from alternative.\n'; wget $APTKEY_STORE_HTTPS -O - | sudo apt-key add -) || (echo 'Fetching apt key by an alternative method failed too. Exiting since ROS cannot be installed.'; error))
lsb_release -a
sudo apt-get update || (echo "ERROR: apt server not responding. This is a rare situation, and usually just waiting for a while clears this. See https://github.com/ros-industrial/industrial_ci/pull/56 for more of the discussion"; error)
sudo apt-get -qq install -y python-catkin-tools python-rosdep python-wstool ros-$ROS_DISTRO-rosbash ros-$ROS_DISTRO-rospack
# If more DEBs needed during preparation, define ADDITIONAL_DEBS variable where you list the name of DEB(S, delimitted by whitespace)
if [ "$ADDITIONAL_DEBS" ]; then
    sudo apt-get install -q -qq -y $ADDITIONAL_DEBS
    if [[ $? > 0 ]]; then
        echo "One or more additional deb installation is failed. Exiting."; error
    fi
fi
# MongoDB hack - I don't fully understand this but its for moveit_warehouse
dpkg -s mongodb || echo "ok"; export HAVE_MONGO_DB=$?
if [ $HAVE_MONGO_DB == 0 ]; then
    sudo apt-get -qq remove -y mongodb mongodb-10gen || echo "ok"
    sudo apt-get -qq install -y mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"
fi

travis_time_end  # setup_ros

travis_time_start setup_rosdep

# Setup rosdep
pip --version
rosdep --version
sudo rosdep init
ret_rosdep=1
rosdep update || while [ $ret_rosdep != 0 ]; do sleep 1; rosdep update && ret_rosdep=0 || echo "rosdep update failed"; done

travis_time_end  # setup_rosdep
travis_time_start setup_catkin

## BEGIN: travis' before_install: # Use this to prepare the system to install prerequisites or dependencies ##
# https://github.com/ros/ros_comm/pull/641, https://github.com/jsk-ros-pkg/jsk_travis/pull/110
sudo apt-get -qq install -y ros-$ROS_DISTRO-roslaunch
(cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/641.diff -O /tmp/641.diff; [ "$ROS_DISTRO" == "hydro" ] && sed -i s@items@iteritems@ /tmp/641.diff ; sudo patch -p4 < /tmp/641.diff)

travis_time_end  # setup_catkin

travis_time_start check_version_ros

# Check ROS tool's version
echo -e "\e[0KROS tool's version"
source /opt/ros/$ROS_DISTRO/setup.bash
rosversion roslaunch
rosversion rospack
apt-cache show python-rospkg | grep '^Version:' | awk '{print $2}'

travis_time_end  # check_version_ros

travis_time_start setup_rosws

## BEGIN: travis' install: # Use this to install any prerequisites or dependencies necessary to run your build ##
# Create workspace
mkdir -p ~/ros/ws_$TARGET_REPO_NAME/src
cd ~/ros/ws_$TARGET_REPO_NAME/src
case "$UPSTREAM_WORKSPACE" in
debian)
    echo "Obtain deb binary for upstream packages."
    ;;
file) # When UPSTREAM_WORKSPACE is file, the dependended packages that need to be built from source are downloaded based on $ROSINSTALL_FILENAME file.
    $ROSWS init .
    # Prioritize $ROSINSTALL_FILENAME.$ROS_DISTRO if it exists over $ROSINSTALL_FILENAME.
    if [ -e $TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO ]; then
        # install (maybe unreleased version) dependencies from source for specific ros version
        $ROSWS merge file://$TARGET_REPO_PATH/$ROSINSTALL_FILENAME.$ROS_DISTRO
    elif [ -e $TARGET_REPO_PATH/$ROSINSTALL_FILENAME ]; then
        # install (maybe unreleased version) dependencies from source
        $ROSWS merge file://$TARGET_REPO_PATH/$ROSINSTALL_FILENAME
    fi
    ;;
http://* | https://*) # When UPSTREAM_WORKSPACE is an http url, use it directly
    $ROSWS init .
    $ROSWS merge $UPSTREAM_WORKSPACE
    ;;
esac

# download upstream packages into workspace
if [ -e .rosinstall ]; then
    # ensure that the downstream is not in .rosinstall
    $ROSWS rm $TARGET_REPO_NAME || true
    $ROSWS update
fi
# TARGET_REPO_PATH is the path of the downstream repository that we are testing. Link it to the catkin workspace
ln -s $TARGET_REPO_PATH .

# Disable metapackage
find -L . -name package.xml -print -exec ${ICI_PKG_PATH}/check_metapackage.py {} \; -a -exec bash -c 'touch `dirname ${1}`/CATKIN_IGNORE' funcname {} \;

source /opt/ros/$ROS_DISTRO/setup.bash # ROS_PACKAGE_PATH is important for rosdep
# Save .rosinstall file of this tested downstream repo, only during the runtime on travis CI
if [ ! -e .rosinstall ]; then
    echo "- git: {local-name: $TARGET_REPO_NAME, uri: 'http://github.com/$TRAVIS_REPO_SLUG'}" >> .rosinstall
fi

travis_time_end  # setup_rosws

travis_time_start before_script

## BEGIN: travis' before_script: # Use this to prepare your build for testing e.g. copy database configurations, environment variables, etc.
source /opt/ros/$ROS_DISTRO/setup.bash # re-source setup.bash for setting environmet vairable for package installed via rosdep
if [ "${BEFORE_SCRIPT// }" != "" ]; then sh -c "${BEFORE_SCRIPT}"; fi

travis_time_end  # before_script

travis_time_start rosdep_install

# Run "rosdep install" command. Avoid manifest.xml files if any.
if [ -e ${ICI_PKG_PATH}/rosdep-install.sh ]; then
    ${ICI_PKG_PATH}/rosdep-install.sh
fi

travis_time_end  # rosdep_install

travis_time_start wstool_info
$ROSWS --version
$ROSWS info -t .
cd ../

travis_time_end  # wstool_info

travis_time_start catkin_build

## BEGIN: travis' script: # All commands must exit with code 0 on success. Anything else is considered failure.
source /opt/ros/$ROS_DISTRO/setup.bash # re-source setup.bash for setting environmet vairable for package installed via rosdep
# for catkin
if [ "${TARGET_PKGS// }" == "" ]; then export TARGET_PKGS=`catkin_topological_order ${TARGET_REPO_PATH} --only-names`; fi
if [ "${PKGS_DOWNSTREAM// }" == "" ]; then export PKGS_DOWNSTREAM=$( [ "${BUILD_PKGS_WHITELIST// }" == "" ] && echo "$TARGET_PKGS" || echo "$BUILD_PKGS_WHITELIST"); fi
if [ "$BUILDER" == catkin ]; then catkin build -i -v --summarize  --no-status $BUILD_PKGS_WHITELIST $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS            ; fi

travis_time_end  # catkin_build

if [ "$NOT_TEST_BUILD" != "true" ]; then
    travis_time_start catkin_run_tests

    # Patches for rostest that are only available in newer codes.
    # Some are already available via DEBs so that patches for them are not needed, but because EOLed distros (e.g. Hydro) where those patches are not released into may be still tested, all known patches are applied here.
    if [ "$ROS_DISTRO" == "hydro" ]; then
        (cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/611.diff -O - | sudo patch -f -p4 || echo "ok" )
        (cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros/pull/82.diff -O - | sudo patch -p4)
        (cd /opt/ros/$ROS_DISTRO/share; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/611.diff -O - | sed s@.cmake.em@.cmake@ | sed 's@/${PROJECT_NAME}@@' | sed 's@ DEPENDENCIES ${_rostest_DEPENDENCIES})@)@' | sudo patch -f -p2 || echo "ok")
    fi

    if [ "$BUILDER" == catkin ]; then
        source devel/setup.bash ; rospack profile # force to update ROS_PACKAGE_PATH for rostest
        catkin run_tests -iv --no-deps --no-status $PKGS_DOWNSTREAM $CATKIN_PARALLEL_TEST_JOBS --make-args $ROS_PARALLEL_TEST_JOBS --
        catkin_test_results build || error
    fi

    travis_time_end  # catkin_run_tests
fi


if [ "$NOT_TEST_INSTALL" != "true" ]; then

    travis_time_start catkin_install_build

    # Test if the packages in the downstream repo build.
    if [ "$BUILDER" == catkin ]; then
        catkin clean --yes
        catkin config --install
        catkin build -i -v --summarize --no-status $BUILD_PKGS_WHITELIST $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS
        source install/setup.bash
        rospack profile
    fi

    travis_time_end  # catkin_install_build
    travis_time_start catkin_install_run_tests

    export EXIT_STATUS=0
    # Test if the unit tests in the packages in the downstream repo pass.
    if [ "$BUILDER" == catkin ]; then
      for pkg in $PKGS_DOWNSTREAM; do
        echo "[$pkg] Started testing..."
        rostest_files=$(find install/share/$pkg -iname '*.test')
        echo "[$pkg] Found $(echo $rostest_files | wc -w) tests."
        for test_file in $rostest_files; do
          echo "[$pkg] Testing $test_file"
          rostest $test_file || export EXIT_STATUS=$?
          if [ $? != 0 ]; then
            echo -e "[$pkg] Testing again the failed test: $test_file.\e[31m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m"
            rostest --text $test_file
            echo -e "[$pkg] Testing again the failed test: $test_file.\e[31m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\e[0m"
          fi
        done
      done
      [ $EXIT_STATUS -eq 0 ] || error  # unless all tests pass, raise error
    fi

    travis_time_end  # catkin_install_run_tests

fi

travis_time_start after_script

## BEGIN: travis' after_script
PATH=/usr/local/bin:$PATH  # for installed catkin_test_results
PYTHONPATH=/usr/local/lib/python2.7/dist-packages:$PYTHONPATH

if [ "${ROS_LOG_DIR// }" == "" ]; then export ROS_LOG_DIR=~/.ros/test_results; fi # http://wiki.ros.org/ROS/EnvironmentVariables#ROS_LOG_DIR
if [ "$BUILDER" == catkin -a -e $ROS_LOG_DIR ]; then $CATKIN_TEST_RESULTS_CMD --all $ROS_LOG_DIR || error; fi
if [ "$BUILDER" == catkin -a -e ~/ros/ws_$TARGET_REPO_NAME/build/ ]; then $CATKIN_TEST_RESULTS_CMD --all ~/ros/ws_$TARGET_REPO_NAME/build/ || error; fi
if [ "$BUILDER" == catkin -a -e ~/.ros/test_results/ ]; then $CATKIN_TEST_RESULTS_CMD --all ~/.ros/test_results/ || error; fi

travis_time_end  # after_script

cd $TARGET_REPO_PATH  # cd back to the repository's home directory with travis
pwd

HIT_ENDOFSCRIPT=true
success  # Process needs to continue to allow custom post process scripts. See https://github.com/ros-industrial/industrial_ci/pull/89 
