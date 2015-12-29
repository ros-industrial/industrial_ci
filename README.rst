# industrial_ci
Continuous integration repository for ROS-Industrial

.. contents:: Table of Contents
   :depth: 3

Description
============

This repository contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`_ configuration that can be commonly used by the repositories in `ros-industrial <https://github.com/ros-industrial>`_ organization (calling them as "**client**" repos). ROS-powered repositories in other organizations can potentially utilize the CI config here too.

(As of November 2015) The CI config in this repository is intended to be used by the client repos by `git submodule` feature. This repo provides configuration for `travis CI`. In client repos you can define custom, repository-specific checks, in addition to the generic config stored in this repo.

What are checked?
------------------------------------

* If your package builds.
* If your package gets installed (i.e. built artifact goes into the `install` space).
* If available tests pass in the package. Because tests use software from `install` space, it is important the building step ends without issues (otherwise tests may not be reached).
* If tests in designated downstream packages pass.

Variables you can configure
------------------------------------

You can configure the behavior in `.travis.yml` in your client repository.

* OS to use. Defined at `dist` tag.

Required environment variables:

* `ROS_REPOSITORY_PATH`: Location of ROS' binary repositories where depended packages get installed from (typically both standard repo and `"Shadow-Fixed" repository <http://wiki.ros.org/ShadowRepository>`_)
* `ROS_DISTRO`: Version of ROS (Indigo, Jade etc.).

Optional environment variables
++++++++++++++++++++++++++++++++

Note that some of these currently tied only to a single option, but we still leave them for the future when more options become available (e.g. ament with BUILDER).

* `ADDITIONAL_DEBS` (default: not set): More DEBs to be used. List the name of DEB(s delimitted by whitespace if multiple DEBs specified). Needs to be full-qualified Ubuntu package name. E.g.: "ros-indigo-roslint ros-indigo-gazebo-ros" (without quotation).
* `BUILD_PKGS` (not set): `PKGS_DOWNSTREAM` will be filled with packages specified with this. Also these packages are to be built when `NOT_TEST_INSTALL` is set.
* `BUILDER` (catkin): Currently only `catkin` is implemented (and with that `catkin_tools` is used instead of `catkin_make`. See `this discussion <https://github.com/ros-industrial/industrial_ci/issues/3>`_).
* `CI_PARENT_DIR` (.ci_config): (NOT recommended to specify) This is the folder name that is used in downstream repositories in order to point to this repo.
* `NOT_TEST_INSTALL` (not set): If you do NOT want to test `install` space, set this as true.
* `PKGS_DOWNSTREAM`: Packages in downstream to be tested. By default, `TARGET_PKGS` is used if set, if not then `BUILD_PKGS` is used.
* `ROS_PARALLEL_JOBS` (-j8): Maximum number of packages which could be built in parallel. See for more detail `documentation of catkin_tools <https://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#full-command-line-interface>`_ that this env variable is passed to internally.
* `ROS_PARALLEL_TEST_JOBS` (not set): Maximum number of packages which could be examined in parallel during the test run. If not set it's filled by `ROS_PARALLEL_JOBS`.
* `ROSWS` (wstool): Currently only `wstool` is available.
* `TARGET_PKGS` (not set): Used to fill `PKGS_DOWNSTREAM` if it is not set. If not set packages are set using the output of `catkin_topological_order` for the source space.
* `USE_DEB`: (NOT Implemented yet) When this is true, the dependended packages that need to be built from source are downloaded based on .travis.rosinstall file.

You see some `*PKGS*` variables. These make things very flexible but in normal usecases you don't need to be bothered with them - just keep them blank.

Usage
======

Here are some operations in your client repositories.

To start using CI config stored in this repo
--------------------------------------------------

With the following few short steps, you can start in your client repository using CI confiurations stored in here (`industrial_ci` repository).

1. Don't forget to activate CI on your github repository (you may do so on https://travis-ci.org/profile/YOUR_GITHUB_ORGANIZATION or https://travis-ci.org/profile/YOUR_GITHUB_USER).

2. In `.travis.yml` file in your client repo, add in `install` section a sentence `git clone https://github.com/ros-industrial/industrial_ci.git .ci_config`, like below:

::

  install:
    - git clone https://github.com/ros-industrial/industrial_ci.git .ci_config
  script:
    - source .ci_config/travis.sh

* Note that `.ci_config` is the required name of the cloned folder; it is hardcoded so you need to use this name.
* Example of entire file `.travis.yml` can be found in `industrial_core/.travis.yml <https://github.com/ros-industrial/industrial_core/.travis.yml>`_.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
----------------------------------------------------------------------------------

Nothing.
Once you add `git clone` statement in your client repo, basically you don't need to do anything to apply the change in `industrial_ci` repository.

Examples
-------------------------------


++++++++++++++++++++++++++++++++

Example client packages
++++++++++++++++++++++++++++++++

* `ros-industrial/industrial_core <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`_
* `ros-industrial-consortium/descartes <https://github.com/ros-industrial-consortium/descartes/blob/indigo-devel/.travis.yml>`_

(Optional but recommended) Subscribe to the change in this repo (industrial_ci)
---------------------------------------------------------------------------------

Because of the aforementioned responsibility for the maintainers to watch the changes in `industrial_ci`, `you're encouraged to subscribe to the updates in this repository <https://github.com/ros-industrial/industrial_ci/subscription>`_.

Add repository-specific CI config in addition
----------------------------------------------------------------

Sometimes CI config stored in `industrial_ci` repo may not be sufficient for your purpose. In that case you can add your own config, while you still take advantage of `industrial_ci` repository.

1. In `.travis.yml` file in your client repo, add the portion below:

::

  script: 
    - source .ci_config/travis.sh
    - source ./travis.sh

2. Create `travis.sh` file and define the checks you wish to add. NOTE: this `.sh` file you add here is a normal shell script, so this shouldn't be written in `travis CI` grammar.

(Optional) To use specific version of industrial_ci in your client repo
-------------------------------------------------------------------------------------

(A minor) downside of how you associate your client repo to this `industrial_ci` repository is that you have no control over which version to use (see `discussion in this ticket <https://github.com/ros-industrial/industrial_ci/issues/3>`_). If you wish you can specify the version. The following is an example using `git submodule`.

First time you define the dependency to this repo
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

1. Run git submodule command.

::

  CLIENTREPO_LOCAL$ git submodule add https://github.com/ros-industrial/industrial_ci .ci_config

This standard `git submodule` command:

* hooks up your client repository to this repo by the name "`.ci_config`" (this name is hardcoded and mandatory).
* stores the configuration in a file called `.gitmodules`.

2. Don't forget to activate CI on your github repository (you may do so on https://travis-ci.org/profile/YOUR_GITHUB_USER).

3. In `.travis.yml` file in your client repo, add the portion below:

::

  script: 
    - source .ci_config/travis.sh
    #- source ./travis.sh  # Optional. Explained later

Also, the example of entire file `.travis.yml` can be found in `industrial_core/.travis.yml <https://github.com/ros-industrial/industrial_core/.travis.yml>`_.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Maintainers of client repos are responsible for applying the changes that happen in this repos, if they want to use up-to-date checks; since `git submodule` does NOT provide features to automatically detect the changes made in the sub modules, maintainers need to keep an eye on the changes.

1. Update the SHA key of the commit in this repo. The command below assumes that there's `.gitmodules` file that's generated by `git submodule add` command explained above.

::

  CLIENTREPO_LOCAL$ git submodule foreach git pull origin master

2. Don't forget to commit the changes the command above makes.


For maintainers of industrial_ci repository
================================================

Checks for industrial_ci repo itself
---------------------------------------

While this repository provides CI config that can be used by other repositories, it also checks this repo itself using the same CI config and the simplest package setting. That is why this repo contains the ROS package files and a test (`CMakeLists.txt`, `package.xml`, `.test`).
