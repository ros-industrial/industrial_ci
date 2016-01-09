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
## Greatly inspired by JSK travis with additional comments for maintainability: https://github.com/jsk-ros-pkg/jsk_travis 
## Author: Isaac I. Y. Saito

## This is a "common" script that can be run on travis CI at a downstream github repository.
## See ./README.rst for the detailed usage.

set -x

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID\e[34m>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\e[0m"
    set -x
}

function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME\e[${_COLOR}m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\e[0m"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME took $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

function error {
    travis_time_end 31
    trap - ERR
    exit 1
}

BUILDER=catkin
ROSWS=wstool
CI_PARENT_DIR=.ci_config  # This is the folder name that is used in downstream repositories in order to point to this repo.

trap error ERR

git branch --all
if [ "`git diff origin/master FETCH_HEAD $CI_PARENT_DIR`" != "" ] ; then DIFF=`git diff origin/master FETCH_HEAD $CI_PARENT_DIR | grep .*Subproject | sed s'@.*Subproject commit @@' | sed 'N;s/\n/.../'`; (cd $CI_PARENT_DIR/;git log --oneline --graph --left-right --first-parent --decorate $DIFF) | tee /tmp/$$-travis-diff.log; grep -c '<' /tmp/$$-travis-diff.log && exit 1; echo "ok"; fi

travis_time_start setup_ros

# Define some config vars
export CI_SOURCE_PATH=$(pwd)
export DOWNSTREAM_REPO_NAME=${PWD##*/}
if [ ! "$CATKIN_PARALLEL_TEST_JOBS" ]; then export CATKIN_PARALLEL_TEST_JOBS="$CATKIN_PARALLEL_JOBS"; fi
if [ ! "$ROS_PARALLEL_JOBS" ]; then export ROS_PARALLEL_JOBS="-j8"; fi
if [ ! "$ROS_PARALLEL_TEST_JOBS" ]; then export ROS_PARALLEL_TEST_JOBS="$ROS_PARALLEL_JOBS"; fi
# If not specified, use ROS Shadow repository http://wiki.ros.org/ShadowRepository
if [ ! "$ROS_REPOSITORY_PATH" ]; then export ROS_REPOSITORY_PATH="http://packages.ros.org/ros-shadow-fixed/ubuntu"; fi
echo "Testing branch $TRAVIS_BRANCH of $DOWNSTREAM_REPO_NAME"
# Set apt repo
sudo -E sh -c 'echo "deb $ROS_REPOSITORY_PATH `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
# Common ROS install preparation
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
lsb_release -a
sudo apt-get update
sudo apt-get install -y -q -qq python-catkin-tools python-rosdep python-wstool ros-$ROS_DISTRO-rosbash ros-$ROS_DISTRO-rospack
# If more DEBs needed during preparation, define ADDITIONAL_DEBS variable where you list the name of DEB(S, delimitted by whitespace)
if [ "$ADDITIONAL_DEBS" ]; then sudo apt-get install -q -qq -y $ADDITIONAL_DEBS;  fi
# MongoDB hack - I don't fully understand this but its for moveit_warehouse
dpkg -s mongodb || echo "ok"; export HAVE_MONGO_DB=$?
if [ $HAVE_MONGO_DB == 0 ]; then
    sudo apt-get remove -q -qq -y mongodb mongodb-10gen || echo "ok"
    sudo apt-get install -q -qq -y mongodb-clients mongodb-server -o Dpkg::Options::="--force-confdef" || echo "ok"
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
sudo apt-get install -y -q -qq ros-$ROS_DISTRO-roslaunch
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
mkdir -p ~/ros/ws_$DOWNSTREAM_REPO_NAME/src
cd ~/ros/ws_$DOWNSTREAM_REPO_NAME/src
# When USE_DEB is true, the dependended packages that need to be built from source are downloaded based on .travis.rosinstall file.
### Currently disabled
###if [ "$USE_DEB" == false ]; then
###    $ROSWS init .
###    if [ -e $CI_SOURCE_PATH/.travis.rosinstall ]; then
###        # install (maybe unreleased version) dependencies from source
###        $ROSWS merge file://$CI_SOURCE_PATH/.travis.rosinstall
###    fi
###    if [ -e $CI_SOURCE_PATH/.travis.rosinstall.$ROS_DISTRO ]; then
###        # install (maybe unreleased version) dependencies from source for specific ros version
###        $ROSWS merge file://$CI_SOURCE_PATH/.travis.rosinstall.$ROS_DISTRO
###    fi
###    $ROSWS update
###    $ROSWS set $DOWNSTREAM_REPO_NAME http://github.com/$TRAVIS_REPO_SLUG --git -y
###fi
# CI_SOURCE_PATH is the path of the downstream repository that we are testing. Link it to the catkin workspace
ln -s $CI_SOURCE_PATH .
####if [ "$USE_DEB" == source -a -e $DOWNSTREAM_REPO_NAME/setup_upstream.sh ]; then $ROSWS init .; $DOWNSTREAM_REPO_NAME/setup_upstream.sh -w ~/ros/ws_$DOWNSTREAM_REPO_NAME ; $ROSWS update; fi
# Disable metapackage
find -L . -name package.xml -print -exec ${CI_SOURCE_PATH}/$CI_PARENT_DIR/check_metapackage.py {} \; -a -exec bash -c 'touch `dirname ${1}`/CATKIN_IGNORE' funcname {} \;

source /opt/ros/$ROS_DISTRO/setup.bash # ROS_PACKAGE_PATH is important for rosdep
# Save .rosinstall file of this tested downstream repo, only during the runtime on travis CI
if [ ! -e .rosinstall ]; then
    echo "- git: {local-name: $DOWNSTREAM_REPO_NAME, uri: 'http://github.com/$TRAVIS_REPO_SLUG'}" >> .rosinstall
fi

travis_time_end  # setup_rosws

travis_time_start before_script

## BEGIN: travis' before_script: # Use this to prepare your build for testing e.g. copy database configurations, environment variables, etc.
source /opt/ros/$ROS_DISTRO/setup.bash # re-source setup.bash for setting environmet vairable for package installed via rosdep
if [ "${BEFORE_SCRIPT// }" != "" ]; then sh -c "${BEFORE_SCRIPT}"; fi

travis_time_end  # before_script

travis_time_start rosdep_install

# Run "rosdep install" command. Avoid manifest.xml files if any.
if [ -e ${CI_SOURCE_PATH}/$CI_PARENT_DIR/rosdep-install.sh ]; then 
    ${CI_SOURCE_PATH}/$CI_PARENT_DIR/rosdep-install.sh
fi

travis_time_end  # rosdep_install

$ROSWS --version
$ROSWS info -t .
cd ../

travis_time_start catkin_build

## BEGIN: travis' script: # All commands must exit with code 0 on success. Anything else is considered failure.
source /opt/ros/$ROS_DISTRO/setup.bash # re-source setup.bash for setting environmet vairable for package installed via rosdep
# for catkin
if [ "${TARGET_PKGS// }" == "" ]; then export TARGET_PKGS=`catkin_topological_order ${CI_SOURCE_PATH} --only-names`; fi
if [ "${PKGS_DOWNSTREAM// }" == "" ]; then export PKGS_DOWNSTREAM=$( [ "${BUILD_PKGS// }" == "" ] && echo "$TARGET_PKGS" || echo "$BUILD_PKGS"); fi
if [ "$BUILDER" == catkin ]; then catkin build -i -v --summarize  --no-status $BUILD_PKGS $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS            ; fi

travis_time_end  # catkin_build

if [ "$NOT_TEST_BUILD" != "true" ]; then
    travis_time_start catkin_run_tests
    
    # Patches for rostest that are only available in newer codes.
    # Some are already available via DEBs so that patches for them are not needed, but because EOLed distros (e.g. Hydro) where those patches are not released into may be still tested, all known patches are applied here.
    (cd /opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages; wget --no-check-certificate https://patch-diff.githubusercontent.com/raw/ros/ros_comm/pull/611.diff -O - | sudo patch -f -p4 || echo "ok" )
    if [ "$ROS_DISTRO" == "hydro" ]; then
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
        catkin clean -a
        catkin config --install
        catkin build -i -v --summarize --no-status $BUILD_PKGS $CATKIN_PARALLEL_JOBS --make-args $ROS_PARALLEL_JOBS
        source install/setup.bash
        rospack profile
        rospack plugins --attrib=plugin nodelet
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
if [ "$BUILDER" == catkin -a -e $ROS_LOG_DIR ]; then catkin_test_results --verbose --all $ROS_LOG_DIR || error; fi
if [ "$BUILDER" == catkin -a -e ~/ros/ws_$DOWNSTREAM_REPO_NAME/build/ ]; then catkin_test_results --verbose --all ~/ros/ws_$DOWNSTREAM_REPO_NAME/build/ || error; fi
if [ "$BUILDER" == catkin -a -e ~/.ros/test_results/ ]; then catkin_test_results --verbose --all ~/.ros/test_results/ || error; fi

travis_time_end  # after_script

cd $TRAVIS_BUILD_DIR  # cd back to the repository's home directory with travis
pwd

