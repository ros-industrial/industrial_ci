# industrial_ci
Continuous integration repository for ROS-Industrial

.. contents:: Table of Contents
   :depth: 3

Introduction
============

This repository contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`_ configuration that can be commonly used by the repositories in `ros-industrial <https://github.com/ros-industrial>`_ organization (calling them as "**client**" repos). ROS-powered repositories in other organizations can potentially utilize the CI config here too.

(As of December 2015) The CI config in this repository is intended to be used by the client repos by `git clone` feature. This repo provides configuration for `Travis CI`. In client repos you can define custom, repository-specific checks, in addition to the generic configs stored in this repo.

FAQ
======

- Q- This config can be used ONLY by the repositories under `github/ros-industrial <https://github.com/ros-industrial>`_ organization?

  A- No. `industrial_ci` repo is open to public. Anyone can use this from any platform. Note that because as of Dec. 2015 it has only config for `Travis CI <https://travis-ci.org/>`_, you may want to use it where Travis CI is available (`github.com` works the best.

- Q- What kind of checks are implemented that are specific to industrial robotics?

  A- As of Dec. 2015, no particular configuration for industrial robot is defined.

- Q- So, can the config be used against any robotics repository?

  A- I'd say no. It's still limited for the projects based on `ROS <http://ros.org/>`_. And checks are run on Ubuntu linux.

- Q- In my project there aren't yet test cases. Can I still have it checked using `industrial_ci` and what can I get out of the check?

  A- The `industrial_ci` still provides valuable checks; it ensures if your package builds without issues. Also installation rules if you define. Just as a headsup that making test cases are highly recommended as your ear may hurt.

What are checked?
------------------------------------

List of the checked items, in the actual order to be run.

1. If your package builds.
2. If available tests pass in the package. Because tests use software from `install` space, it is important the building step ends without issues (otherwise tests may not be reached).
3. If your package gets installed (i.e. built artifact goes into the `install` space).
4. If tests in designated downstream packages pass.

Your client repository does NOT need to pass all of above steps; in fact you can have only some of them tested. To pass the steps without having tested, simply "empty" them. For instance, in your client repository:

* Step 2 will be skipped when no test files are present.
* Step 3 will be skipped when no installation rule is defined.
* Step 4 will be skipped when no downstream packages to be tested are defined.

Prerequisite
============

In order for your repository to get checked with configurations in `industrial_ci`, it needs:

* To be a `Catkin package <http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage>`_ (uses CMake for build configuration), since many checks are triggered by the `Catkin`-based commands.
* Build-able on Linux (as of Dec 2015, Ubuntu 14.04/Trusty is used). Although your repository is not necessarilly intended for Linux, checks are run on Linux.

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
* Example of entire file `.travis.yml` can be found in `industrial_core/.travis.yml <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`_.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
----------------------------------------------------------------------------------

Nothing.
Once you add `git clone` statement in your client repo, basically you don't need to do anything to apply the change in `industrial_ci` repository.

Example client packages
-------------------------------

* `ros-industrial/industrial_core <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`_
* `ros-industrial-consortium/descartes <https://github.com/ros-industrial-consortium/descartes/blob/indigo-devel/.travis.yml>`_

Variables you can configure
------------------------------------

You can configure the behavior in `.travis.yml` in your client repository.

* OS to use. Defined at `dist` tag.

Required environment variables:

* `ROS_REPOSITORY_PATH`: Location of ROS' binary repositories where depended packages get installed from (typically both standard repo (`http://packages.ros.org/ros/ubuntu`) and `"Shadow-Fixed" repository <http://wiki.ros.org/ShadowRepository>`_ (`http://packages.ros.org/ros-shadow-fixed/ubuntu`))
* `ROS_DISTRO`: Version of ROS in all lower case. E.g.: `indigo` / `jade`

Optional environment variables
++++++++++++++++++++++++++++++++

Note that some of these currently tied only to a single option, but we still leave them for the future when more options become available (e.g. ament with BUILDER).

* `ADDITIONAL_DEBS` (default: not set): More DEBs to be used. List the name of DEB(s delimitted by whitespace if multiple DEBs specified). Needs to be full-qualified Ubuntu package name. E.g.: "ros-indigo-roslint ros-indigo-gazebo-ros" (without quotation).
* `BEFORE_SCRIPT`: (default: not set): Used to specify shell commands that run before building packages.
* `BUILD_PKGS` (default: not set): `PKGS_DOWNSTREAM` will be filled with packages specified with this. Also these packages are to be built when `NOT_TEST_INSTALL` is set.
* `BUILDER` (default: catkin): Currently only `catkin` is implemented (and with that `catkin_tools` is used instead of `catkin_make`. See `this discussion <https://github.com/ros-industrial/industrial_ci/issues/3>`_).
* `CATKIN_PARALLEL_JOBS` (default: -p4): Maximum number of packages to be built in parallel that is passed to underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools`. See for more detail about `number of build jobs <http://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#controlling-the-number-of-build-jobs>`_ and `documentation of catkin_tools <https://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#full-command-line-interface>`_ that this env variable is passed to internally in `catkin-tools`.
* `CATKIN_PARALLEL_TEST_JOBS` (default: -p4): Maximum number of packages which could be examined in parallel during the test run. If not set it's filled by `ROS_PARALLEL_JOBS`.
* `CI_PARENT_DIR` (default: .ci_config): (NOT recommended to specify) This is the folder name that is used in downstream repositories in order to point to this repo.
* `NOT_TEST_BUILD` (default: not set): If true, tests in build space won't be run.
* `NOT_TEST_INSTALL` (default: not set): If true, tests in `install` space won't be run.
* `PRERELEASE` (default: false): If `true`, run `Prerelease Test on docker that emulates ROS buildfarm <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`_. The usage of Prerelease Test feature is `explained more in this section <https://github.com/ros-industrial/industrial_ci/blob/add/dockerbased_prerelease/README.rst#optional-run-ros-prerelease-test>`_.
* `PRERELEASE_DOWNSTREAM_DEPTH` (0 to 4, default: 0): Number of the levels of the package dependecies the Prerelease Test targets at. Range of the level is defined by ROS buildfarm (`<http://prerelease.ros.org>`_). NOTE: a job can run exponentially longer for the values greater than `0` depending on how many packages depend on your package (and remember a job on Travis CI can only run for up to 50 minutes).
* `PRERELEASE_REPONAME` (default: not set): The target of Prerelease Test (that you select at `<http://prerelease.ros.org/indigo>`_, `<http://prerelease.ros.org/kinetic>`_ etc.) If not set then it tests the package of the repository's name. You can specify this by your ROS package name format (with underscore e.g. `industrial_core`), not Debian package name format. NOTE that this package name must be listed in the `rosdistro/distribution.yaml` (e.g. [for ROS Indigo](https://github.com/ros/rosdistro/blob/master/indigo/distribution.yaml)) (this requirement comes from ROS buildfarm's Prerelease Test).
* `PKGS_DOWNSTREAM` (default: explained): Packages in downstream to be tested. By default, `TARGET_PKGS` is used if set, if not then `BUILD_PKGS` is used.
* `ROS_PARALLEL_JOBS` (default: -j8): Maximum number of packages to be built in parallel by the underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).
* `ROS_PARALLEL_TEST_JOBS` (default: -j8): Maximum number of packages which could be examined in parallel during the test run by the underlining build tool. If not set it's filled by `ROS_PARALLEL_JOBS`. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).
* `ROSINSTALL_FILENAME` (default: not set): See `UPSTREAM_WORKSPACE` description.
* `ROSWS` (default: wstool): Currently only `wstool` is available.
* `TARGET_PKGS` (default: not set): Used to fill `PKGS_DOWNSTREAM` if it is not set. If not set packages are set using the output of `catkin_topological_order` for the source space.
* `UPSTREAM_WORKSPACE` (default: debian): When this is set `file`, the dependended packages that need to be built from source are downloaded based on a `.rosinstall` file in your repository. Use `$ROSINSTALL_FILENAME` to specify the file name. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#optional-build-depended-packages-from-source>`_.
* `USE_DEB` (*DEPRECATED*: use `UPSTREAM_WORKSPACE` instead. default: true): if `true`, `UPSTREAM_WORKSPACE` will be set as `debian`. if `false`, `file` will be set. See `UPSTREAM_WORKSPACE` section for more info.

Note: You see some `*PKGS*` variables. These make things very flexible but in normal usecases you don't need to be bothered with them - just keep them blank.

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

(Optional) Run ROS Prerelease Test
-------------------------------------------------------------------------------------

Running `docker-based ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`_ is strongly recommended when you make a release. There are, however, some inconvenience (requires host computer setup, runs on your local host, etc. Detail discussed in `a ticket <https://github.com/ros-industrial/industrial_ci/pull/35#issue-150581346>`_). `industrial_ci` provides a way to run it on your `Travis CI` test.

To do so, add a single line to your Travis config (eg. `.travis.yml`):

::

  ROS_DISTRO=indigo PRERELEASE=true

Or with more configuration:

::

  ROS_DISTRO=indigo PRERELEASE=true PRERELEASE_REPONAME=industrial_core PRERELEASE_DOWNSTREAM_DEPTH=0

NOTE: A job that runs Prerelease Test does not run the checks that are defined in `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`_. To run both, use `matrix` in Travis config.

See the usage sample in `.travis in indusrial_ci repository <https://github.com/ros-industrial/industrial_ci/blob/master/.travis.yml>`_.

The following is some tips to be shared for running Prerelease Test on Travis CI using `industrial_ci`.

(Workaround) Don't want to always run Prerelease Test
+++++++++++++++++++++++++++++++++++++++++++++++++++++

The jobs that run Prerelease Test may usually take longer than the tests defined in `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`_, which can result in longer time for the entire Travis jobs to finish. This is usually okay, as developers who are concerned with PRs might not wait for the Travis result that eagerly (besides that, Travis CI limits the maximum run time as 50 minutes so there can't be very long run). If you're concerned, however, then you may want to separately run the Prerelease Test. An example way to do this is to create a branch specifically for Prerelease Test where `.travis.yml` only defines a check entry with `PRERELEASE` turned on. E.g.:

::

  :
  env:
    matrix:
      - ROS_DISTRO=indigo PRERELEASE=true
  :

Then open a pull request using this branch against the branch that the change is subject to be merged. You do not want to actually merge this branch no matter what the Travis result is. This branch is solely for Prerelease Test purpose.

(Optional) Build depended packages from source
----------------------------------------------

By default the packages your package depend upon are installed via binaries. However, you may want to build them via source in some cases (e.g. when depended binaries are not available). There are a few ways to do so in `industrial_ci`. Examples of all are available in `.travis.yml file on this repository <https://github.com/ros-industrial/industrial_ci/blob/master/.travis.yml>`_.

Use .rosinstall file to specify the depended packages source repository
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Standard way is 1) set `UPSTREAM_WORKSPACE` as `file`, 2) create a file `$ROSINSTALL_FILENAME` using the same file format as `.rosinstall <http://docs.ros.org/independent/api/rosinstall/html/rosinstall_file_format.html>`_ and place it at the top level directory of your package.

Have multiple .rosinstall files per ROS-distro
++++++++++++++++++++++++++++++++++++++++++++++

By adding `.$ROS_DISTRO` suffix to your `$ROSINSTALL_FILENAME` file, you can specify which file to use per your `$ROS_DISTRO`. So the syntax of the file name for this purpose is `$ROSINSTALL_FILENAME.$ROS_DISTRO`.
For example, let's say you want to test multiple distros (indigo, jade) and you have `.travis.rosinstall` and `.travis.rosinstall.jade` files in your repo. You can define the Travis config as:

::

    env:
      matrix:

        - ROS_DISTRO=indigo UPSTREAM_WORKSPACE=file
        - ROS_DISTRO=jade   UPSTREAM_WORKSPACE=file

With this config, for indigo default file name `.travis.rosinstall` will be seached and used if found. For jade, the file that consists of the default file name plus `.jade` suffix will be prioritized.

When `$ROSINSTALL_FILENAME.$ROS_DISTRO` file isn't found, `$ROSINSTALL_FILENAME` will be used for all jobs that define `UPSTREAM_WORKSPACE`.

Use .rosinstall from external location
++++++++++++++++++++++++++++++++++++++++++++++

You can utilize `.rosinstall` file stored anywhere as long as its location is URL specifyable. To do so, set its complete path URL directly to `UPSTREAM_WORKSPACE`.

For maintainers of industrial_ci repository
================================================

Checks for industrial_ci repo itself
---------------------------------------

While this repository provides CI config that can be used by other repositories, it also checks this repo itself using the same CI config and the simplest package setting. That is why this repo contains the ROS package files and a test (`CMakeLists.txt`, `package.xml`, `.test`).
