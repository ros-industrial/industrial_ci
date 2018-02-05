================
Industrial CI
================
CI (Continuous Integration) configuration for `ROS` (`Robot Operating System <http://ros.org>`_).

.. contents:: Table of Contents
   :depth: 2

Detailed documentation
========================

Other than the brief introduction in this page, you can also check `the detailed doc here <./doc/index.rst>`_.

Introduction
============

This package contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`_ configuration that any ROS-powered packages can commonly use.
Some notable feature:

* Checks if your package builds, installs without issues. If unit/system tests are defined run them. `ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest>`_ can optionally be run.
* Proven to cover the general requirements of the ROS-based robotics repositories. Easily configurable.
* Users can add custom pre/post processes.
* Covers ROS Indigo, Jade, Kinetic distribution.
* As of January 2017, this repo provides configuration for `Travis CI` only, although it should be easily deployed from other CI services.

For a brief overall introduction, you could also check a presentation:

* `ROS-Industrial community meeting <http://rosindustrial.org/news/2016/6/14/ros-i-community-web-meeting-june-2016>`_

Quick Start
============

With a few steps, you can start in your client repository using CI confiurations stored in `industrial_ci`.

1. Activate CI on your github repository.

- For `Travis CI <https://travis-ci.org/>`_), you may do so either at https://travis-ci.org/profile/YOUR_GITHUB_ORGANIZATION or at https://travis-ci.org/profile/YOUR_GITHUB_USER (depending on where your repository sits).

2. In `.travis.yml` file in your client repo, add in "`install`" section a sentence `git clone https://github.com/ros-industrial/industrial_ci.git .ci_config`, like below:

::

  install:
    - git clone https://github.com/ros-industrial/industrial_ci.git .ci_config
  script:
    - .ci_config/travis.sh

* Note: The name `.ci_config` is NO longer REQUIRED for the cloned folder starting version 0.3.2; you can pick any name as long as the folder is hidden (by being prepended by ".").

Concrete examples of config files
-------------------------------------

- For development branch intended for ROS Indigo: `ros_canopen <https://github.com/ros-industrial/ros_canopen/blob/0a42bf181804167834b8dc3b80bfca971f24546f/.travis.yml>`_
- For development branch intended for ROS Indigo onward:
   - `example 1 <https://github.com/ros-industrial/industrial_core/blob/eeb6a470e05233d0efaaf8c32a9e4133cdcbb80b/.travis.yml>`_ (Indigo and Jade compatible).
   - `example 2 <https://github.com/ros-drivers/leap_motion/blob/954924befd2a6755f9d310f4a8b57aa526056a80/.travis.yml>`_ (Indigo, Jade, Kinetic compatible. Also runs `ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest>`_).
- For development branch intended for ROS Kinetic: `industrial_core <https://github.com/ros-industrial/industrial_core/blob/a07f9089b0f6c8a931bab80b7fca959dd6bba05b/.travis.yml>`_
- For more complexed example: `.travis.yml <https://github.com/ros-industrial/industrial_ci/blob/d09b8dd40d7f1fa1ad5b62323a1d6b2ca836e558/.travis.yml>`_ from the same repo. You can see how options are used.

Metrics
========

There might not an easy way to precisely count how many repositories out there are using `industrial_ci`. Counting that number isn't even our priority at all, but we're often simply curious. Here's some ways that give us some clues for the usage metrics:

- `Searching Github repos that contain string industrial_ci <https://github.com/search?p=1&q=industrial_ci+-repo%3Aros-industrial%2Findustrial_ci&ref=searchresults&type=Code&utf8=%E2%9C%93>`_) (with some duplicates. Excluding industrial_ci repo):

  - 457 (Dec 12, 2017)
  - 142 (Jan 20, 2017)

- Github--> `Graphs` --> `Traffic` view (visible only to admins).

  - Dec 12, 2017

  .. figure:: doc/industrial_ci_traffic_20171212.png

  - Jan 20, 2017

  .. figure:: doc/industrial_ci_traffic_20170120.png

EoF
