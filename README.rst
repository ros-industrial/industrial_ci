================
Industrial CI
================
.. image:: https://travis-ci.org/ros-industrial/industrial_ci.svg?branch=master
    :target: https://travis-ci.org/ros-industrial/industrial_ci
    :alt: Travis CI status
.. image:: https://gitlab.com/ipa-mdl/industrial_ci/badges/master/pipeline.svg
    :target: https://gitlab.com/ipa-mdl/industrial_ci/commits/master
    :alt: Gitlab CI status
.. image:: https://img.shields.io/badge/License-Apache%202.0-blue.svg
    :target: https://opensource.org/licenses/Apache-2.0
    :alt: License

CI (Continuous Integration) configuration for `ROS` (`Robot Operating System <http://ros.org>`__).
This is the refactored version with ROS2 support, the old verson can be found in the `legacy branch <https://github.com/ros-industrial/industrial_ci/tree/legacy>`__.
Please check the `migration guide <doc/migration_guide.md>`__ as well.

.. contents:: Table of Contents
   :depth: 2

Detailed documentation
========================

Other than the brief introduction in this page, you can also check `the detailed doc here <./doc/index.rst>`__.

Introduction
============

This package contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`__ scripts that any ROS-powered packages can commonly use.
Some notable feature:

* Checks if your package builds, installs without issues. If unit/system tests are defined run them. `ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest>`__ can optionally be run.
* Proven to cover the general requirements of the ROS-based robotics repositories. Easily configurable.
* Users can add custom pre/post processes.
* Covers ROS1 Indigo, Jade, Kinetic, Lunar, Melodic and ROS2 distributions.
* This repo provides scripts for `Bitbucket CI`, `Gitlab CI`, and `Travis CI` only, but it can be easily adapted for other CI services.

For a brief overall introduction, you could also check a presentation:

* `ROS-Industrial community meeting <http://rosindustrial.org/news/2016/6/14/ros-i-community-web-meeting-june-2016>`__

Quick Start
============

With a few steps, you can start in your client repository using CI confiurations stored in `industrial_ci`.

For Travis CI
--------------

1. Activate CI for your github repository on `Travis CI <https://travis-ci.org/>`__). You may do so either at https://travis-ci.org/profile/YOUR_GITHUB_ORGANIZATION or at https://travis-ci.org/profile/YOUR_GITHUB_USER (depending on where your repository sits).

2. Add `.travis.yml` file to your repository root (`complete template <https://github.com/ros-industrial/industrial_ci/blob/master/doc/.travis.yml>`__):

::

  language: generic
  services:
    - docker

  env:
    matrix:
      - ROS_DISTRO="indigo"

  install:
    - git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci.git .industrial_ci -b master
  script:
    - .industrial_ci/travis.sh

* Note: The name `.industrial_ci` is NO longer REQUIRED for the cloned folder starting version 0.3.2; you can pick any name (recommended practice to keep the folder hidden (by prepending ".").

For Gitlab CI
-------------

1. Enable CI for your repo. Please refer to `official doc <https://docs.gitlab.com/ee/ci/quick_start/>`__ for the steps to do so. Note for Gitlab CI, necessary steps might be different between hosted version (i.e. the one on gitlab.com) v.s. the one on your own server, which Gitlab doesn't always clarify in its documentation.
  1. For your server version, enable a runner for your Gitlab project which uses the Docker executor. See instructions on how to `install <https://docs.gitlab.com/runner/install/index.html>`__ and `register <https://docs.gitlab.com/runner/register/index.html>`__ such a runner with your Gitlab instance if you haven't done so yet.
1. In `.gitlab-ci.yml` file in your client repo, add the following minimal configuration (this snippet can be the entire content of the file), replacing indigo for your chosen distro:

::

   image: docker:git
   services:
     - docker:dind
   before_script:
     - apk add --update bash coreutils tar
     - git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci .industrial_ci -b master
   indigo:
     script: .industrial_ci/gitlab.sh ROS_DISTRO=indigo


For Bitbucket Pipelines
-----------------------

1. Enable CI for your repo. Please refer to `official doc <https://confluence.atlassian.com/bitbucket/get-started-with-bitbucket-pipelines-792298921.html>`__ for the steps to do so.
2. In the `bitbucket-pipelines.yml` file in your client repo, add the following minimal configuration (this snippet can be the entire content of the file), replacing indigo for your chosen distro:

::

   image: docker:git

   pipelines:
     default:
        - step:
            services:
              - docker
            script:
              - apk add --update bash coreutils tar
              - git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci .industrial_ci -b master
              - .industrial_ci/bitbucket.sh ROS_DISTRO=indigo

   definitions:
     services:
       docker:
         memory: 2048


Concrete examples of config files
-------------------------------------

- A `template for Travis CI <doc/.travis.yml>`__.
- For development branch intended for ROS Indigo: `ros_canopen <https://github.com/ros-industrial/ros_canopen/blob/0a42bf181804167834b8dc3b80bfca971f24546f/.travis.yml>`__
- For development branch intended for ROS Indigo onward:
   - `example 1 <https://github.com/ros-industrial/industrial_core/blob/eeb6a470e05233d0efaaf8c32a9e4133cdcbb80b/.travis.yml>`__ (Indigo and Jade compatible).
   - `example 2 <https://github.com/ros-drivers/leap_motion/blob/954924befd2a6755f9d310f4a8b57aa526056a80/.travis.yml>`__ (Indigo, Jade, Kinetic compatible. Also runs `ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest>`__).
- For development branch intended for ROS Kinetic: `industrial_core <https://github.com/ros-industrial/industrial_core/blob/a07f9089b0f6c8a931bab80b7fca959dd6bba05b/.travis.yml>`__
- For more complexed example: `.travis.yml <https://github.com/ros-industrial/industrial_ci/blob/d09b8dd40d7f1fa1ad5b62323a1d6b2ca836e558/.travis.yml>`__ from the same repo. You can see how options are used.
- For Gitlab CI, a small `sample config <./.gitlab-ci.yml>`__.

Metrics
========

There might not an easy way to precisely count how many repositories out there are using `industrial_ci`. Counting that number isn't even our priority at all, but we're often simply curious. Here's some ways that give us some clues for the usage metrics:

- `Searching Github repos that contain string industrial_ci <https://github.com/search?p=1&q=industrial_ci+-repo%3Aros-industrial%2Findustrial_ci&ref=searchresults&type=Code&utf8=%E2%9C%93>`__) (with some duplicates. Excluding industrial_ci repo):

  - 1,841 (Jan 2, 2019)
  - 675 (May 15, 2018)
  - 457 (Dec 12, 2017)
  - 142 (Jan 20, 2017)

- Github--> `Graphs` --> `Traffic` view (visible only to admins).

  - Dec 30, 2018

  .. figure:: http://ros-industrial.github.io/industrial_ci/images/industrial_ci_traffic_20181230.png

  - May 15, 2018

  .. figure:: http://ros-industrial.github.io/industrial_ci/images/industrial_ci_20180515_traffic.png

  - Dec 12, 2017

  .. figure:: http://ros-industrial.github.io/industrial_ci/images/industrial_ci_traffic_20171212.png

  - Jan 20, 2017

  .. figure:: http://ros-industrial.github.io/industrial_ci/images/industrial_ci_traffic_20170120.png

EoF
