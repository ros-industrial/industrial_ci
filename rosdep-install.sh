#!/bin/bash -x

trap 'find -L . -name manifest.xml.deprecated | xargs -n 1 -i dirname {} | xargs -n 1 -i mv `pwd`/{}/manifest.xml.deprecated `pwd`/{}/manifest.xml' 1 2 3 15

find -L . -name package.xml -exec dirname {} \; | xargs -n 1 -i find {} -name manifest.xml | xargs -n 1 -i mv {} {}.deprecated # rename manifest.xml for rosdep install
sudo rosdep install -r -q -n --from-paths . `for i in ${ROS_PACKAGE_PATH//:/\ /}; do if [ -e $i ]; then echo -n $i' '; fi; done` --ignore-src --rosdistro $ROS_DISTRO -y
find -L . -name manifest.xml.deprecated | xargs -n 1 -i dirname {} | xargs -n 1 -i mv `pwd`/{}/manifest.xml.deprecated `pwd`/{}/manifest.xml
