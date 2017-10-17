================
Industrial CI
================
Continuous integration repository for ROS-Industrial

.. contents:: Table of Contents
   :depth: 3

Introduction
============

This repository contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`_ configuration that can be commonly used by the repositories in `ros-industrial <https://github.com/ros-industrial>`_ organization. Non ros-industrial repositories in other organizations can utilize the CI config here too, as long as they are ROS-powered.

As of December 2015, this repo provides configuration for `Travis CI`. The CI config in this repository is intended to be obtained by `git clone` feature. In client repos you can define custom, repository-specific checks, in addition to the generic configs stored in this repo.

For a brief introduction, you could also check a presentation:

* `ROS-Industrial community meeting <http://rosindustrial.org/news/2016/6/14/ros-i-community-web-meeting-june-2016>`_

Supported ROS distributions
----------------------------------

Following `ROS distributions <http://wiki.ros.org/action/login/Distributions>`_ are supported.

* `Indigo <http://wiki.ros.org/indigo>`_
* `Jade <http://wiki.ros.org/jade>`_
* `Kinetic <http://wiki.ros.org/kinetic>`_
* `Lunar <http://wiki.ros.org/lunar>`_

Terminology
----------------

* **client repository**: The repositories that use the configuration stored in this repo to run CI jobs.
* **downstream packages**: The software packages that depend on the package that's targetted to be tested using industrial_ci.

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

- Q- My package uses a custom Point Cloud Library (PCL) version or the `industrial_calibration <https://github.com/ros-industrial/industrial_calibration>`_ package, how do I make build work?

  A- You can check `advanced_industrial_ci <https://github.com/InstitutMaupertuis/advanced_industrial_ci>`_ which provides scripts and binaries for PCL 1.8.0 and the Ceres solver. An integration example can be found `here <https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/.travis.yml>`_.

- Q- How does the target package get installed?

  A- `Travis CI` does this. It pulls in your package to an running instance of an operating system of your choice, and place your package under `/home/travis`.

- Q- The jobs on `Travis CI` are failing. How can I fix them?

  A- (1) Find the section where error occurred that caused CI to stop. Sections are folded nicely and it's colored red when it fails. (2) Then identify whether the issue is pertaining to your package, or something else. Sometimes a cause is in `industrial_ci`, not your package. (3) Reviewing `Common Build Problems for Travis CI <https://docs.travis-ci.com/user/common-build-problems>`_ helps you to isolate the root cause. (4) If you think the root cause is in `industrial_ci`, (or if you're not sure,) ask at its `issue tracker <https://github.com/ros-industrial/industrial_ci/issues>`_.

- Q- How can I customize the jobs?

  A- (1) There are a number of variables to customize your jobs that you can learn the usage `in this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#variables-you-can-configure>`_. (2) You can define pre- and post-processes, in addition to the default scripts (it's `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`_ for `Travis CI`). See `this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-pre-post-process-custom-commands>`_ for how.

What are checked?
------------------------------------

List of the checked items, in the actual order to be run.

1. If your package builds.
2. If available tests in the given package pass. Because tests use software from `install` space, it is important that the building step ends without issues (otherwise the tests may not be reached).
3. If your package gets installed (i.e. built artifact goes into the `install` space).
4. If downstream packages are designated, the tests in those packages pass.

Your client repository does NOT need to pass all of above steps; in fact you can have only some of them tested. To pass the steps without having tested, simply "empty" them. For instance, in your client repository:

* Step 2 will be skipped when no test files are present.
* Step 3 will be skipped when no installation rule is defined.
* Step 4 will be skipped when no downstream packages to be tested are defined.

Prerequisite
============

In order for your repository to get checked with configurations in `industrial_ci`, it needs:

* To be a `Catkin package <http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage>`_ (uses CMake for build configuration), since many checks are triggered by the `Catkin`-based commands.
* Build-able on Linux (as of Dec 2015, Ubuntu 14.04/Trusty is used). Although your repository is not necessarilly intended for Linux, checks are run on Linux.

Basic Usage
===========

Here are some operations in your client repositories.

To start using CI config stored in this repo
--------------------------------------------------

With the following few short steps, you can start in your client repository using CI confiurations stored in here (`industrial_ci` repository).

1. Don't forget to activate Travis CI on your github repository (you may do so on https://travis-ci.org/profile/YOUR_GITHUB_ORGANIZATION or https://travis-ci.org/profile/YOUR_GITHUB_USER).

2. In `.travis.yml` file in your client repo, add in `before_config` section a sentence `git clone https://github.com/ros-industrial/industrial_ci.git .ci_config`, like below:

::

  before_config:
    - git clone https://github.com/ros-industrial/industrial_ci.git .ci_config
  script:
    - .ci_config/travis.sh

* Note that `.ci_config` is the required name of the cloned folder; it is hardcoded so you need to use this name.
* Example of entire file `.travis.yml` can be found in `industrial_core/.travis.yml <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`_.
* A Gitlab CI config can be found in `.gitlab-ci.yml <https://github.com/ros-industrial/industrial_ci/blob/master/.gitlab-ci.yml>`_.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
----------------------------------------------------------------------------------

Nothing.
Once you add `git clone` statement in your client repo, basically you don't need to do anything to apply the change in `industrial_ci` repository.

Example client packages
-------------------------------

* `ros-industrial/industrial_core <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`_
* `ros-industrial-consortium/descartes <https://github.com/ros-industrial-consortium/descartes/blob/indigo-devel/.travis.yml>`_

Advanced Usage
==============

Variables you can configure
------------------------------------

You can configure the behavior in `.travis.yml` in your client repository.

* OS to use. Defined at `dist` tag.

Required environment variables:

* `ROS_DISTRO`: Version of ROS in all lower case. E.g.: `indigo` / `jade`

Optional environment variables
++++++++++++++++++++++++++++++++

Note that some of these currently tied only to a single option, but we still leave them for the future when more options become available (e.g. ament with BUILDER).

* `ADDITIONAL_DEBS` (default: not set): More DEBs to be used. List the name of DEB(s delimitted by whitespace if multiple DEBs specified). Needs to be full-qualified Ubuntu package name. E.g.: "ros-indigo-roslint ros-indigo-gazebo-ros" (without quotation).
* `AFTER_SCRIPT`: (default: not set): Used to specify shell commands that run after all source tests. NOTE: `Unlike Travis CI <https://docs.travis-ci.com/user/customizing-the-build#Breaking-the-Build>`_ where `after_script` doesn't affect the build result, the result in the commands specified with this DOES affect the build result.
* `BEFORE_SCRIPT`: (default: not set): Used to specify shell commands that run before building packages.
* `BUILD_PKGS_WHITELIST` (default: not set): Packages to be built can be explicitly specified with this, in ROS package name format (i.e. using underscore. No hyphen). This is useful when your repo contains some packages that you don't want to be used upon testing. Downstream packages, if necessary, should be also specified using this. Also these packages are to be built when `NOT_TEST_INSTALL` is set. Finally, packages specified with this will be built together with those speicified using unimplmented `USE_DEB`.
* `BUILDER` (default: catkin): Currently only `catkin` is implemented (and with that `catkin_tools` is used instead of `catkin_make`. See `this discussion <https://github.com/ros-industrial/industrial_ci/issues/3>`_).
* `CATKIN_CONFIG` (default: not set): `catkin config --install` is used by default and with this variable you can 1) pass additional config options, or 2) overwrite `--install` by `--no-install`. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-customize-catkin-config>`_.
* `CATKIN_LINT` (default: not set): If true, run `catkin_lint <http://fkie.github.io/catkin_lint/>`_ with `--explain` option.
* `CATKIN_LINT_ARGS` (default: not set): If true, you can pass whatever argument(s) `catkin_lint` takes, except `--explain` that is set by default. Options can be delimit by space if passing multiple. Adding `--strict` is recommended, with which a CI job fails if there's any error and/or warning occurs. It runs between build and install
* `CATKIN_PARALLEL_JOBS` (default: -p4): Maximum number of packages to be built in parallel that is passed to underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools`. See for more detail about `number of build jobs <http://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#controlling-the-number-of-build-jobs>`_ and `documentation of catkin_tools <https://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#full-command-line-interface>`_ that this env variable is passed to internally in `catkin-tools`.
* `CATKIN_PARALLEL_TEST_JOBS` (default: -p4): Maximum number of packages which could be examined in parallel during the test run. If not set it's filled by `ROS_PARALLEL_JOBS`.
* `CCACHE_DIR` (default: not set): If set, `ccache <https://en.wikipedia.org/wiki/Ccache>`_ gets enabled for your build to speed up the subsequent builds in the same job if anything. See `detail. <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#cache-build-artifacts-to-speed-up-the-subsequent-builds-if-any>`_
* `CI_PARENT_DIR` (default: .ci_config): (NOT recommended to specify) This is the folder name that is used in downstream repositories in order to point to this repo.
* `DEBUG_BASH` (default: not set): If set with any value (e.g. `true`), all executed commands that are not printed by default to reduce print space will be printed.
* `DOCKER_BASE_IMAGE` (default: $OS_NAME:$OS_CODE_NAME): Base image used for building the CI image. Could be used to pre-bundle dependecies or to run tests for different architectures. See `this PR <https://github.com/ros-industrial/industrial_ci/pull/174>`_ for more info.
* `DOCKER_IMAGE` (default: not set): Selects a Docker images different from default one. Please note, this disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO` as ROS needs already to be installed in the image.
* `DOCKER_FILE` (default: not set): Instead of pulling an images from the Docker hub, build it from the given path or URL. Please note, this disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO`, they have to be set in the build file instead.
* `DOCKER_BUILD_OPTS` (default: not set): Used do specify additional build options for Docker.
* `DOCKER_RUN_OPTS` (default: not set): Used do specify additional run options for Docker.
* `EXPECT_EXIT_CODE` (default: 0): exit code must match this value for test to succeed
* `NOT_TEST_BUILD` (default: not set): If true, tests in `build` space won't be run.
* `NOT_TEST_INSTALL` (default: not set): If true, tests in `install` space won't be run.
* `OS_CODE_NAME` (default: derived from ROS_DISTRO): See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`_.
* `OS_NAME` (default: ubuntu): Possible options: {`ubuntu`, `debian`}. See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`_.
* `PRERELEASE` (default: false): If `true`, run `Prerelease Test on docker that emulates ROS buildfarm <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`_. The usage of Prerelease Test feature is `explained more in this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-ros-prerelease-test>`_.
* `PRERELEASE_DOWNSTREAM_DEPTH` (0 to 4, default: 0): Number of the levels of the package dependecies the Prerelease Test targets at. Range of the level is defined by ROS buildfarm (`<http://prerelease.ros.org>`_). NOTE: a job can run exponentially longer for the values greater than `0` depending on how many packages depend on your package (and remember a job on Travis CI can only run for up to 50 minutes).
* `PRERELEASE_REPONAME` (default: TARGET_REPO_NAME): The  name of the target of Prerelease Test in rosdistro (that you select at `<http://prerelease.ros.org>`_). You can specify this if your repository name differs from the corresponding rosdisto entry. See `here <https://github.com/ros-industrial/industrial_ci/pull/145/files#r108062114>`_ for more usage.
* `PKGS_DOWNSTREAM` (default: explained): Packages in downstream to be tested. By default, `TARGET_PKGS` is used if set, if not then `BUILD_PKGS` is used.
* `ROS_PARALLEL_JOBS` (default: -j8): Maximum number of packages to be built in parallel by the underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).
* `ROS_PARALLEL_TEST_JOBS` (default: -j8): Maximum number of packages which could be examined in parallel during the test run by the underlining build tool. If not set it's filled by `ROS_PARALLEL_JOBS`. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).
* `ROS_REPO` (default: ros-shadow-fixed): `ROS_REPO` can be used to set `ROS_REPOSITORY_PATH` based on known aliases: 'ros`/`main`, 'ros-shadow-fixed`/`testing` or `building`.
* `ROS_REPOSITORY_PATH`: Location of ROS' binary repositories where depended packages get installed from (typically both standard repo (`http://packages.ros.org/ros/ubuntu`) and `"Shadow-Fixed" repository <http://wiki.ros.org/ShadowRepository>`_ (`http://packages.ros.org/ros-shadow-fixed/ubuntu`)). Since version 0.3.4, `ROS_REPO` is recommended, and `ROS_REPOSITORY_PATH` is for more intermediate usage only (e.g. to specify your own binary repository (non-standard / in house)). Backward compatibility is preserved.
* `ROSDEP_SKIP_KEYS` (default: not set): space-separated list of keys that should get skipped by `rosdep install`.
* `ROSINSTALL_FILENAME` (default: .travis.rosinstall): Only used when `UPSTREAM_WORKSPACE` is set to `file`. See `UPSTREAM_WORKSPACE` description.
* `ROSWS` (default: wstool): Currently only `wstool` is available.
* `TARGET_PKGS` (default: not set): Used to fill `PKGS_DOWNSTREAM` if it is not set. If not set packages are set using the output of `catkin_topological_order` for the source space.
* `UPSTREAM_WORKSPACE` (default: debian): When set as `file`, the dependended packages that need to be built from source are downloaded based on a `.rosinstall` file in your repository. Use `$ROSINSTALL_FILENAME` to specify the file name. When set to a URL, downloads the rosinstall configuration from an ``http`` location. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#optional-build-depended-packages-from-source>`_.
* `USE_DEB` (*DEPRECATED*: use `UPSTREAM_WORKSPACE` instead. default: true): if `true`, `UPSTREAM_WORKSPACE` will be set as `debian`. if `false`, `file` will be set. See `UPSTREAM_WORKSPACE` section for more info.
* `USE_MOCKUP` (default: not set): reletive path to mockup packages to be used for the tests
* `VERBOSE_OUTPUT` (default: not set): If `true`, build tool (e.g. Catkin) output prints in verbose mode.

Note: You see some `*PKGS*` variables. These make things very flexible but in normal usecases you don't need to be bothered with them - just keep them blank.

Use custom Docker images
------------------------

As you see in the `optional variables section <./index.rst#optional-environment-variables>`_, there are a few different ways to specify `Docker` image if you like. Here are some more detail:

Pulling Docker image from an online hub
+++++++++++++++++++++++++++++++++++++++

You can pull any `Docker` image by specifying in `DOCKER_IMAGE` variable, as long as the following requirement is met:

* `python-catkin-tools`, `python-pip`, `python-rosdep`, `python-wstool`
* sources.list set up (`example <http://wiki.ros.org/kinetic/Installation/Ubuntu#Installation.2BAC8-Ubuntu.2BAC8-Sources.Setup_your_sources.list>`_).

If your Docker image is missing any of the above libraries, then you can still pass their name by `ADDITIONAL_DEBS` (see `variables section <./index.rst#optional-environment-variables>`_).

Note-1. This disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO` as ROS needs already to be installed in the image.

Note-2. For some images, `ROS_DISTRO` variable still needs to be set. This holds for `ROS official Docker images <https://hub.docker.com/_/ros/>`_ as of Sept. 2017.

(Optional but recommended) Subscribe to the change in this repo (industrial_ci)
---------------------------------------------------------------------------------

Because of the aforementioned responsibility for the maintainers to watch the changes in `industrial_ci`, `you're encouraged to subscribe to the updates in this repository <https://github.com/ros-industrial/industrial_ci/subscription>`_.

Run ROS Prerelease Test
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

(Optional) Customize `catkin config`
------------------------------------

By default, `industrial_ci` builds packages with `catkin config --install`, which requires `install` rules to pass CI jobs. This might not be suitable in some cases, e.g. with your experimental packages where you have no plan to make them deployable so that `install` rules are nothing but extra burden. Also, you may want to add addtional configuration for `catkin config`. In these cases define "`CATKIN_CONFIG`" variable.

Example-1::

  CATKIN_CONFIG='--no-install'

This allows you to use `devel` space for the job, instead of `install` space.

Example-2::

  CATKIN_CONFIG='-DMyCustomBuildFlag=true'

This will end up defining the following CMake arg. `install` space is still used::

  Additional CMake Args:       -DMyCustomBuildFlag=true

Reference:

 * `Discussion about install space <https://github.com/ros-industrial/industrial_ci/issues/54>`_
 * `Detail for catkin config <http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html>`_ for more info about `catkin-tools`.

Cache build artifacts to speed up the subsequent builds (if any)
----------------------------------------------------------------

If `CCACHE_DIR` is set (not set by default), `ccache <https://en.wikipedia.org/wiki/Ccache>`_ gets enabled for your build to speed up the subsequent builds in the same job if anything.
Recommended value is `$HOME/.ccache`, but any non-used directory works.

https://docs.travis-ci.com/user/caching/#Arbitrary-directories

 * Enable cache. How to do so depends on the CI system of your choice.

   On Travis CI, add as follows (`refrence <https://docs.travis-ci.com/user/caching/#Arbitrary-directories>`_)::

    cache:
      directories:
        - $HOME/.ccache  # can be any valid cache location
  

 * Define `CCACHE_DIR` variable. You can apply to all of your jobs by something like below::

  env:
    global:
      - CCACHE_DIR=$HOME/.ccache
    matrix:
      :

Or define `CCACHE_DIR` per job.

NOTE:
  * Beware, if you use `run_ci <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#id39>`_, the files will be owned by root!
  * Caching may not work for packages with "smaller" number of files (see also `this discussion <https://github.com/ros-industrial/industrial_ci/pull/182>`_).
    
Add repository-specific CI config in addition
----------------------------------------------------------------

Sometimes CI config stored in `industrial_ci` repo may not be sufficient for your purpose. In that case you can add your own config, while you still take advantage of `industrial_ci` repository.

1. In `.travis.yml` file in your client repo, add the portion below:

::

  script:
    - .ci_config/travis.sh
    - ./travis.sh

2. Create `travis.sh` file and define the checks you wish to add. NOTE: this `.sh` file you add here is a normal shell script, so this shouldn't be written in `travis CI` grammar.

To use specific version of industrial_ci in your client repo
-------------------------------------------------------------------------------------

(A minor) downside of how you associate your client repo to this `industrial_ci` repository is that you have no control over which version to use (see `discussion in this ticket <https://github.com/ros-industrial/industrial_ci/issues/3>`_). If you wish you can specify the version.

The following is an example using `git submodule`. Note that when using this method, you have to manually update the `submodule` every time there's an update in this `industrial_ci` package.

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
    - .ci_config/travis.sh
    #- ./travis.sh  # Optional. Explained later

Also, the example of entire file `.travis.yml` can be found in `industrial_core/.travis.yml <https://github.com/ros-industrial/industrial_core/.travis.yml>`_.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Maintainers of client repos are responsible for applying the changes that happen in this repos, if they want to use up-to-date checks; since `git submodule` does NOT provide features to automatically detect the changes made in the sub modules, maintainers need to keep an eye on the changes.

1. Update the SHA key of the commit in this repo. The command below assumes that there's `.gitmodules` file that's generated by `git submodule add` command explained above.

::

  CLIENTREPO_LOCAL$ git submodule foreach git pull origin master

2. Don't forget to commit the changes the command above makes.

Run pre/post-process custom commands
-----------------------------------------

You may want to add custom steps prior to the setup defined in `./travis.sh <./travis.sh>`_. Example usecases:

* A device driver package X in your repository or in your repository's dependency requires a prorietary library installed. This library is publicly available, but not via apt or any package management system and thus the only way you can install it is in a classic way (unzip, run installer etc.) (`More discussion <https://github.com/ros-industrial/industrial_ci/issues/14>`_).

* You want to run `ros_lint` (`thi discussion <https://github.com/ros-industrial/industrial_ci/issues/58#issuecomment-223601916>`_ may be of your interest).

In such cases, you can specify the script(s) in `BEFORE_SCRIPT` and/or `AFTER_SCRIPT` variables. For example::

  env:
    global:
      - BEFORE_SCRIPT='./your_custom_PREprocess.sh'
      - AFTER_SCRIPT='./your_custom_POSTprocess.sh'
  script:
    - .ci_config/travis.sh

NOTE: If you specify scripts in `script` section without using aforementioned variables, those will be run directly on CI, not on the `Docker` where `.ci_config/travis.sh` runs on.::

  script:
    - ./your_custom_PREprocess.sh  <-- Runs on CI server natively.
    - .ci_config/travis.sh         <-- Runs on Docker on CI server.
    - ./your_custom_POSTprocess.sh <-- Runs on CI server natively.

(Optional) Build depended packages from source
----------------------------------------------

By default the packages your package depend upon are installed via binaries. However, you may want to build them via source in some cases (e.g. when depended binaries are not available). There are a few ways to do so in `industrial_ci`; By utilizing `rosinstall <http://docs.ros.org/independent/api/rosinstall/html/>`_, you can specify the packages that you want to be built from source.

Note that while building the designated packages from source, other packages are resolved still from binary automatically by `rosdep <http://wiki.ros.org/rosdep>`_.

Examples of how to enable all of the following cases are available in `.travis.yml file on this repository <https://github.com/ros-industrial/industrial_ci/blob/master/.travis.yml>`_.

Use .rosinstall file to specify the depended packages source repository
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

WARNING: In all cases where you want to utilize `.rosinstall` (or similar name) files, be sure to set `USE_DEB` as `false`, or simply not define it.

For using a rosinstall file located locally within the repository, define one or two variables as:

1) set `UPSTREAM_WORKSPACE` as `file`.
2) optionally create a file `$ROSINSTALL_FILENAME` using the same file format as `.rosinstall <http://docs.ros.org/independent/api/rosinstall/html/rosinstall_file_format.html>`_ and place it at the top level directory of your package. Its file name is your choice (typically this file is prefixed with a dot).

Example. This expects a file `.travis.rosinstall` available at the top directory of the repository being tested::

    :
    - ROS_DISTRO=indigo  UPSTREAM_WORKSPACE=file
    :

Another example. Now you're specifying the file name as `.your_rosinstall`::

    :
    - ROS_DISTRO=indigo  UPSTREAM_WORKSPACE=file  $ROSINSTALL_FILENAME=".your_rosinstall"
    :

For using a rosinstall file located externally from the repository:

1) set `UPSTREAM_WORKSPACE` to some URL, for example: `https://github.com/ros-planning/moveit_docs/blob/jade-devel/moveit.rosinstall`
2) do not specify `$ROSINSTALL_FILENAME`

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

(Optional) Type of OS and distribution
--------------------------------------

Ubuntu and its distro are guessed by default from ROS_DISTRO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

You can specify the OS and its distribution to run the CI job by setting `OS_NAME` and `OS_CODE_NAME`.
By default users don't need to set this and its value will be automatically guessed according to the value of `ROS_DISTRO`. e.g.::

  `ROS_DISTRO=indigo`  --> `OS_NAME=ubuntu` `OS_CODE_NAME=trusty`
  `ROS_DISTRO=jade`    --> `OS_NAME=ubuntu` `OS_CODE_NAME=trusty`
  `ROS_DISTRO=kinetic` --> `OS_NAME=ubuntu` `OS_CODE_NAME=xenial`
  `ROS_DISTRO=lunar`   --> `OS_NAME=ubuntu` `OS_CODE_NAME=xenial`

Use non-default Ubuntu distro
+++++++++++++++++++++++++++++

E.g. `OS_CODE_NAME=yakkety` or `zesty` for ROS Lunar are available.

Use Debian
++++++++++

E.g.::

  `OS_NAME=debian` `OS_CODE_NAME=jessie`
  `OS_NAME=debian` `OS_CODE_NAME=stretch`

All combinations available of OS and distros
++++++++++++++++++++++++++++++++++++++++++++++

Possible combination of `OS_NAME` and `OS_CODE_NAME` depend on available Docker images. See `ros-industrial/docker/ci <https://github.com/ros-industrial/docker/tree/master/ci>`_.

Checking older ROS distros with industrial_ci
--------------------------------------------------------

For the older ROS distributions than `those that are supported <https://github.com/ros-industrial/industrial_ci#supported-ros-distributions>`_, you may still be able to use `industrial_ci`. Here's how to do so taking ROS `Hydro` as an example.

For `Travis CI`, you need at least the following changes in `.travis.yml`:

* Use `dist: precise` (instead of e.g. "`dist: trusty`").
* Define `ROS_DISTRO` with  `hydro` (so have `ROS_DISTRO="hydro"`).

A successful example from `swri-robotics/mapviz <https://github.com/swri-robotics/mapviz/blob/49b0c5748950a956804e1976cfd7a224fa3f3f7d/.travis.yml>`_.

Run industrial_ci on local host
---------------------------------------

Since version 0.3.3, you can run `industrial_ci` on your local host. This can be useful e.g. when you want to integrate industrial_ci into your CI server.
To do so,

0. `Install Docker <https://docs.docker.com/engine/installation/linux/>`_
1. Build and install industrial_ci (which is a `catkin package <http://wiki.ros.org/ROS/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.What_makes_up_a_catkin_Package.3F>`_). Source setting.
2. Change directory to the package you like to test.
3. Run `run_ci` script with your settings.

Example:

::

  $ cd ~/cws/src && git clone https://github.com/ros-industrial/industrial_ci.git && cd ~/cws
  $ catkin config --install
  $ catkin b industrial_ci
  $ source install/setup.bash
  $ roscd ros_canopen   (or any package you test)
  $ rosrun industrial_ci run_ci ROS_DISTRO=indigo ROS_REPO=ros-shadow-fixed

(ROS_DISTRO could be read from your environment as well)

For maintainers of industrial_ci repository
================================================

Checks for industrial_ci repo itself
---------------------------------------

While this repository provides CI config that can be used by other repositories, it also checks this repo itself using the same CI config and the simplest package setting. That is why this repo contains the ROS package files and a test (`CMakeLists.txt`, `package.xml`, `.test`).
