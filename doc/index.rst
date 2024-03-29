================
Industrial CI
================
Continuous integration repository for ROS-Industrial

.. contents:: Table of Contents
   :depth: 3

Introduction
============

This repository contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`__ scripts that can be commonly used by the repositories in `ros-industrial <https://github.com/ros-industrial>`__ organization. Non ros-industrial repositories in other organizations can utilize the CI scripts here too, as long as they are ROS-powered.

As of November 2019, this repo provides scripts for *Bitbucket CI*, *Gitlab CI*, *GitHub Actions* and *Travis CI*. The CI scripts in this repository are intended to be obtained by ``git clone`` feature. In client repos you can define custom, repository-specific checks, in addition to the generic configs stored in this repo.

For a brief introduction, you could also check a presentation:

* `ROS-Industrial community meeting <http://rosindustrial.org/news/2016/6/14/ros-i-community-web-meeting-june-2016>`__

Supported Platform
------------------

Supported ROS Distributions
+++++++++++++++++++++++++++

The following `ROS <http://wiki.ros.org/Distributions>`__ / `ROS2 <https://index.ros.org/doc/ros2/Releases/>`__  distributions are supported.

* `Indigo <http://wiki.ros.org/indigo>`__ *(EOL)*
* `Jade <http://wiki.ros.org/jade>`__ *(EOL)*
* `Kinetic <http://wiki.ros.org/kinetic>`__ *(EOL)*
* `Lunar <http://wiki.ros.org/lunar>`__ *(EOL)*
* `Melodic <http://wiki.ros.org/melodic>`__ *(EOL)*
* `Noetic <http://wiki.ros.org/noetic>`__
* `Ardent <https://index.ros.org/doc/ros2/Releases/Release-Ardent-Apalone/>`__ *(EOL)*
* `Bouncy <https://index.ros.org/doc/ros2/Releases/Bouncy/>`__ *(EOL)*
* `Crystal <https://index.ros.org/doc/ros2/Releases/Release-Crystal-Clemmys/>`__ *(EOL)*
* `Dashing <https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/>`__ *(EOL)*
* `Eloquent <https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/>`__ *(EOL)*
* `Foxy <https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/>`__ *(EOL)*
* `Galactic <https://docs.ros.org/en/foxy/Releases/Release-Galactic-Geochelone.html>`__ *(EOL)*
* `Humble <https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html>`__
* `Iron <https://docs.ros.org/en/rolling/Releases/Release-Iron-Irwini.html>`__
* `Rolling <https://index.ros.org/doc/ros2/Releases/Release-Rolling-Ridley/>`__

Supported CIs
+++++++++++++

* Bitbucket CI
* Gitlab CI
* GitHub Actions
* Travis CI

As of January 2018, this document uses the format of *Travis CI* by default, unless specified.

Variety of operating system by utilizing Docker
-----------------------------------------------

After `version 0.3.3 <http://docs.ros.org/kinetic/changelogs/industrial_ci/changelog.html>`__, all checks run on *Docker* so that you can have the variety of the operating system to check your software against, freed from the limitation of your CI platform (e.g. as of 2017 on *Travis CI*, Ubuntu 16.04 isn't available yet).

Terminology
----------------

* **CI config**: ``.travis.yml`` for Travis CI. ``.gitlab-ci.yml`` for *Gitlab CI*.
* **client repository**: The repositories that use the configuration stored in this repo to run CI jobs.
* **downstream packages**: The software packages that depend on the package that's targetted to be tested using industrial_ci.
* **merge parent**: The branch that your pull/merge request is opened against.

FAQ
======

- Q- This config can be used ONLY by the repositories under `github/ros-industrial <https://github.com/ros-industrial>`__ organization?

  A- No. ``industrial_ci`` repo is open to public. Anyone can use this from any platform. Note that because as of Dec. 2015 it has only config for `Travis CI <https://travis-ci.org/>`__, you may want to use it where Travis CI is available (``github.com`` works the best.

- Q- What kind of checks are implemented that are specific to industrial robotics?

  A- As of Dec. 2015, no particular configuration for industrial robot is defined.

- Q- So, can the config be used against any robotics repository?

  A- I'd say no. It's still limited for the projects based on `ROS <http://ros.org/>`__. And checks are run on Ubuntu linux.

- Q- In my project there aren't yet test cases. Can I still have it checked using ``industrial_ci`` and what can I get out of the check?

  A- The ``industrial_ci`` still provides valuable checks; it ensures if your package builds without issues. Also installation rules if you define. Just as a headsup that making test cases are highly recommended as your ear may hurt.

- Q- My package uses a custom Point Cloud Library (PCL) version or the `industrial_calibration <https://github.com/ros-industrial/industrial_calibration>`__ package, how do I make build work?

  A- You can check `advanced_industrial_ci <https://github.com/InstitutMaupertuis/advanced_industrial_ci>`__ which provides scripts and binaries for PCL 1.8.0 and the Ceres solver. An integration example can be found `here <https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/.travis.yml>`__.

- Q- How does the target package get installed?

  A- *Travis CI* does this. It pulls in your package to a running instance of an operating system of your choice, and place your package under ``/home/travis``.

- Q- The jobs on *Travis CI* are failing. How can I fix them?

  A- (1) Find the section where error occurred that caused CI to stop. Sections are folded nicely and it's colored red when it fails. (2) Then identify whether the issue is pertaining to your package, or something else. Sometimes a cause is in ``industrial_ci``, not your package. (3) Reviewing `Common Build Problems for Travis CI <https://docs.travis-ci.com/user/common-build-problems>`__ helps you to isolate the root cause. (4) If you think the root cause is in ``industrial_ci``, (or if you're not sure,) ask at its `issue tracker <https://github.com/ros-industrial/industrial_ci/issues>`__.

- Q- How can I customize the jobs?

  A- (1) There are a number of variables to customize your jobs that you can learn the usage `in this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#variables-you-can-configure>`__. (2) You can define pre- and post-processes, in addition to the default scripts (it's `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`__ for *Travis CI*). See `this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-pre-post-process-custom-commands>`__ for how.

What is checked?
------------------------------------

List of the checked items, in the actual order to be run.

1. If your package builds.
2. If available tests in the given package pass. Because tests use software from ``install`` space, it is important that the building step ends without issues (otherwise the tests may not be reached).
3. If your package gets installed (i.e. built artifact goes into the ``install`` space).
4. If downstream packages are designated, the tests in those packages pass.

Your client repository does NOT need to pass all of above steps; in fact you can have only some of them tested. To pass the steps without having tested, simply "empty" them. For instance, in your client repository:

* Step 2 will be skipped when no test files are present.
* Step 3 will be skipped when no installation rule is defined.
* Step 4 will be skipped when no downstream packages to be tested are defined.

Basic Usage
===========

To start using the CI scripts stored in this repo
--------------------------------------------------

With the following few short steps, you can start in your client repository using ``industrial_ci`` scripts.

1. Don't forget to activate CI for your repository.

   * For Travis CI and GitHub, you may do so on https://travis-ci.org/profile/YOUR_GITHUB_ORGANIZATION or https://travis-ci.org/profile/YOUR_GITHUB_USER (replace capital with your value).
2. In `CI config <#terminology>`__ file in your client repo include and run industrial_ci.

   * For Travis create the file `.travis.yml <https://github.com/ros-industrial/industrial_ci/blob/master/doc/.travis.yml>`__.
   * A GitHub Actions config can be found in `github-ci.yml <https://github.com/ros-industrial/industrial_ci/blob/master/doc/industrial_ci_action.yml>`__.
   * A Gitlab CI config can be found in `.gitlab-ci.yml <https://github.com/ros-industrial/industrial_ci/blob/master/.gitlab-ci.yml>`__.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
----------------------------------------------------------------------------------

Nothing.
Once you add ``git clone`` statement in your client repo, basically you don't need to do anything to apply the change in ``industrial_ci`` repository.

Example client packages
-------------------------------

* `ros-industrial/industrial_core <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`__
* `ros-industrial-consortium/descartes <https://github.com/ros-industrial-consortium/descartes/blob/indigo-devel/.travis.yml>`__

Advanced Usage
==============

Variables you can configure
------------------------------------

You can configure the behavior in the `CI config <#terminology>`__ in your client repository.

Required environment variables:

* ``ROS_DISTRO``: Version of ROS in all lower case. E.g.: ``indigo``. If it is set in the custom Docker (base) image, it might be omitted in the script call.

Optional environment variables
++++++++++++++++++++++++++++++++

Note that some of these currently tied only to a single option, but we still leave them for the future when more options become available.

* **ABICHECK_MERGE** (default: not set): Used only when ``ABICHECK_URL`` is set. For *Travis CI* it can be set to 'auto' to auto-detect pull requests. If set to ``true`` the merge parent (see `Terminology section <#terminology>`__) will be checked against.
* **ABICHECK_URL** (default: not set): Run binary compatibility check with `ABICC <https://github.com/lvc/abi-compliance-checker>`__. The URL should point to a baseline archive (\*.tar.\*,\*.zip, \*.tgz or \*.tbz2). See more in `the ABI checks section <#abi-checks>`__)
* **ABICHECK_VERSION** (default: not set): Used only when ``ABICHECK_URL`` is set. Version name (for display only) of the set of code, which the location is specified in ``ABICHECK_URL`` of. The version will be automatically read from the URL passed in ``ABICHECK_URL`` if possible, but for a URL that doesn't point to a version-based file name (e.g. the link for a tagged version on Gitlab doesn't).
* **ADDITIONAL_DEBS** (default: not set): More DEBs to be used. List the name of DEB(s delimitted by whitespace if multiple DEBs specified). Needs to be full-qualified Ubuntu package name. E.g.: ``ros-indigo-roslint ros-indigo-gazebo-ros``
* **AFTER_SCRIPT** (default: not set): Used to specify shell commands that run after all source tests. NOTE: `Unlike Travis CI <https://docs.travis-ci.com/user/customizing-the-build#Breaking-the-Build>`__ where ``after_script`` doesn't affect the build result, the result in the commands specified with this DOES affect the build result. See more `here <./index.rst#run-pre-post-process-custom-commands>`__.
* **APT_PROXY** (default: not set): Configure APT to use the provided URL as http proxy.
* **BASEDIR** (default: ``$HOME``): Base directory in which the upstream, target, and downstream workspaces will be built. Note: this directory is bind-mounted, so it can be read by the CI service, but its contents will not persist in the image configured by ``DOCKER_COMMIT``
* **BLACK_CHECK** (default: not set): If true, will check Python code formatting with `Black <https://black.readthedocs.io/en/stable/>`__.
* **BUILDER** (default: ``catkin_tools`` for ROS1, ``colcon`` for ROS2): Select the builder e.g. to build ROS1 packages with colcon (options: ``catkin_tools``, ``colcon``, ``catkin_make``, ``catkin_make_isolated``).
* **CATKIN_LINT** (default: not set. Value range: [true|pedantic]): If ``true``, run `catkin_lint <http://fkie.github.io/catkin_lint/>`__ with ``--explain`` option. If ``pedantic``, ``catkin_lint`` command runs with ``--strict -W2`` option, i.e. more verbose output will print, and the CI job fails if there's any error and/or warning occurs. Industrial CI uses the `latest version available from pypi <https://pypi.org/project/catkin-lint/>`__. If the older version in the `ros repository <http://packages.ros.org/ros/ubuntu/pool/main/c/catkin-lint/>`__ is required, :code:`ADDITIONAL_DEBS='python-catkin-lint'` can be added to the CI Config.
* **CATKIN_LINT_ARGS** (default: not set): If true, you can pass whatever argument(s) ``catkin_lint`` takes, except ``--explain`` that is set by default. Options can be delimit by space if passing multiple.
* **CC** / **CXX** (default: not set): Environment variables to specify the C/C++ compilers. If required, these can be installed by specifying ``ADDITIONAL_DEBS``. E.g. ``ADDITIONAL_DEBS=clang CC=clang CXX=clang++`` uses Clang to build the workspaces. Note, these are the regular environment variables - and they work as pass-through in ``industrial_ci`` as well.
* **CCACHE_DIR** (default: not set): If set, `ccache <https://en.wikipedia.org/wiki/Ccache>`__ gets enabled for your build to speed up the subsequent builds in the same job if anything. See `detail. <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#cache-build-artifacts-to-speed-up-the-subsequent-builds-if-any>`__
* **CMAKE_ARGS** (default: not set): CMake arguments that get passed to the builder for all workspaces.
* **CLANG_FORMAT_CHECK** (default: not set. Value range: [``<format-style>``|``file``]): If set, run the `clang-format <https://clang.llvm.org/docs/ClangFormat.html>`__ check. Set the argument to ``file`` if the style configuration should be loaded from a ``.clang-format`` file, located in one of the parent directories of the source file.
* **CLANG_FORMAT_VERSION** (default: not set): Version of clang-format to install and use (relates to both the apt package name as well as the executable), e.g., ``CLANG_FORMAT_VERSION=3.8``.
* **CLANG_TIDY** (default: not set. Value range: [``true``|``pedantic``]): If set, run `clang.tidy <https://clang.llvm.org/extra/clang-tidy/>`__ to check the code in all packages and fail in case of errors. If ``pedantic``, warnings will be treated as errors as well.
* **CLANG_TIDY_ARGS** (default: not set): Pass additional arguments to ``clang-tidy``, e.g. ``CLANG_TIDY_ARGS='-checks=modernize-*'``
* **CLANG_TIDY_BASE_REF** (default: not set.): If set, clang-tidy tests will be performed on files only that changed since the given ref. If not set, clang-tidy checks are performed on all files.
  For pull requests, you usually want to (re)test on changed files only. As all CI providers provide corresponding environment variables to recognize a PR, this can be easily configured, e.g. for github actions:

  :push does not check: ``${{ github.base_ref || github.ref }}``
  :push performs full check: ``${{ github.base_ref || '' }}``
  :manually trigger full check: ``${{ github.event_name != 'workflow_dispatch' && (github.base_ref || github.ref) || '' }}``

* **CLANG_TIDY_JOBS** (default: number of processors): Maximum number of parallel jobs that execute ``clang-tidy``. The parallel processing is restricted to per build space (=one ROS package, except for ``BUILDER=catkin_make``)
* **DEBUG_BASH** (default: not set): If set with any value (e.g. ``true``), all executed commands that are not printed by default to reduce print space will be printed.
* **DOCKER_COMMIT** (default: not set): If set, the docker image, which contains the build and test artifacts, will be saved in a Docker image. If unset, the container will not be commited and is removed. The value is used to specify an image name during the ``docker commit`` command. *Note* while this allows you to use the resulting docker image with eg. `docker run -it <DOCKER_COMMIT> /bin/bash`, the main intended use is with the `rerun_ci` feature or subsequent `industrial_ci`runs, which also manages attaching the required volumes etc.
* **DOCKER_COMMIT_MSG** (default: not set): used to specify a commit during the docker commit command which is triggered by setting ``DOCKER_COMMIT``. If unset and if ``DOCKER_COMMIT`` is set then the commit message will be empty. See more ``DOCKER_COMMIT``.
* **DOCKER_IMAGE** (default: not set): Selects a Docker images different from default one. Please note, this disables the handling of ``ROS_REPOSITORY_PATH`` and ``ROS_DISTRO`` as ROS needs already to be installed in the image.
* **DOCKER_PULL** (default: ``true``): set to false if custom docker image should not be pulled, e.g. if it was created locally
* **DOCKER_RUN_OPTS** (default: not set): Used to specify additional run options for Docker.
* **DOWNSTREAM_CMAKE_ARGS** (default: not set): Addtional CMake arguments for downstream `workspace <#workspace-management>`__.
* **DOWNSTREAM_WORKSPACE** (default: not set): Definition of downstream `workspace <#workspace-management>`__.
* **EXPECT_EXIT_CODE** (default: ``0``): exit code must match this value for test to succeed
* **IMMEDIATE_TEST_OUTPUT** (default: not set): If true, test output is printed immediately during the tests
* **NOT_TEST_BUILD** (default: not set): If true, tests in ``build`` space won't be run.
* **NOT_TEST_DOWNSTREAM** (default: not set): If true, tests in the downstream workspace won't be run.
* **OS_CODE_NAME** (default: derived from ROS_DISTRO): See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`__.
* **OS_NAME** (default: derived from OS_CODE_NAME): Possible options: {``ubuntu``, ``debian``}. See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`__.
* **PARALLEL_BUILDS** (default: 0): Sets the number of parallel build jobs among all packages. ``0`` or ``true`` unsets the limit.
* **PARALLEL_TESTS** (default: 1): Sets the number of parallel test jobs. ``0`` or ``true`` unsets the limit.
* **PREFIX** (default: not set): Prefix string or directory for the workspaces created during the build job. The upstream, target, and downstream workspaces will be created at ``$BASEDIR/${PREFIX}<upstream_ws|target_ws|downstream_ws>``.
* **PRERELEASE** (default: ``false``): If ``true``, run `Prerelease Test on docker that emulates ROS buildfarm <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`__. The usage of Prerelease Test feature is `explained more in this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-ros-prerelease-test>`__.
* **PRERELEASE_DOWNSTREAM_DEPTH** (default: ``0``): Number of the levels of the package dependencies the Prerelease Test targets at. Range of the level is defined by ROS buildfarm (`<http://prerelease.ros.org>`__). NOTE: a job can run exponentially longer for the values greater than ``0`` depending on how many packages depend on your package (and remember a job on Travis CI can only run for up to 50 minutes).
* **PRERELEASE_REPONAME** (default: ``$TARGET_REPO_NAME``): The name of the target of Prerelease Test in rosdistro (that you select at `<http://prerelease.ros.org>`__). You can specify this if your repository name differs from the corresponding rosdisto entry. See `here <https://github.com/ros-industrial/industrial_ci/pull/145/files#r108062114>`__ for more usage.
* **ROS_REPO** (default: ``testing``): ``ROS_REPO`` can be used to set ``ROS_REPOSITORY_PATH`` based on known aliases: ``ros``/``main``, ``ros-shadow-fixed``/``testing`` or ``building``. ``ROS_REPO=false`` disables the repository set-up.
* **ROS_REPOSITORY_KEY** (default: not set): Location of ROS' binary repository key; either as URL, file path or fingerprint.
* **ROS_REPOSITORY_PATH**: Location of ROS' binary repositories where depended packages get installed from (typically both standard repo (``http://packages.ros.org/ros/ubuntu``) and `"Shadow-Fixed" repository <http://wiki.ros.org/ShadowRepository>`__ (``http://packages.ros.org/ros-shadow-fixed/ubuntu``)). Since version 0.3.4, ``ROS_REPO`` is recommended, and ``ROS_REPOSITORY_PATH`` is for more intermediate usage only (e.g. to specify your own binary repository (non-standard / in house)). Backward compatibility is preserved.
* **ROSDEP_SKIP_KEYS** (default: not set): space-separated list of keys that should get skipped by ``rosdep install``.
* **ROSDISTRO_INDEX_VERSION** (default: not set): If set, patch the files in /etc/ros/rosdep/sources.list.d/*.list to use this version instead of master and set ROSDISTRO_INDEX_URL accordingly
* **ROSINSTALL_FILENAME** (*deprecated*, default: ``.travis.rosinstall``): Only used when ``UPSTREAM_WORKSPACE`` is set to ``file``. See ``UPSTREAM_WORKSPACE`` description.
* **PYLINT_ARGS** (default: not set): pass command line arguments to ``pylint`` command (e.g. ``--output-format=parseable --errors-only``) - can e.g. be used to ``ignore_modules``
* **PYLINT_CHECK** (default: false): If ``true``, run ``pylint`` checks
* **PYLINT_EXCLUDE** (default: not set): Pattern that can be used to exclude files via the ``-not -path`` filter.
* **TARGET_CMAKE_ARGS** (default: not set): Addtional CMake arguments for target `workspace <#workspace-management>`__.
* **TARGET_WORKSPACE** (default: ``$TARGET_REPO_PATH``): Definition of sources for target `workspace <#workspace-management>`__.
* **UNDERLAY** (default: not set): Path to an install space (instead of ``/opt/ros/$ROS_DISTRO``) to be used as an underlay of the workspaces being set up be ICI, e.g. a workspace provided by a custom docker image
* **UPSTREAM_CMAKE_ARGS** (default: not set): Addtional CMake arguments for upstream `workspace <#workspace-management>`__.
* **UPSTREAM_WORKSPACE** (default: not set): Definition of upstream `workspace <#workspace-management>`__.
* **VERBOSE_OUTPUT** (default: ``false``): If ``true``, build tool (e.g. Catkin) output prints in verbose mode.
* **VERBOSE_TESTS** (default: ``false``): If ``true``, build tool (e.g. Catkin) output prints in verbose mode during ``run_tests`` step.


Workspace management
--------------------

Workflow
++++++++
The default test will just build the packages in the target repository and optionally run the contained tests.
This behavior can be expanded with addtional workspaces

A. Upstream workspace: Source packages that are needed for building or testing the target or downstream packages

   1. Fetch source code (``UPSTREAM_WORKSPACE``)
   2. Install dependencies with ``rosdep``
   3. Build workspace ``$BASEDIR/${PREFIX}upstream_ws``, chained to /opt/ros (or ``UNDERLAY``)

B. Target workspace: Packages in your target repository that should get build and tested

   1. Fetch source code (``TARGET_WORKSPACE``)
   2. Install dependencies with ``rosdep``
   3. Build workspace ``$BASEDIR/${PREFIX}target_ws``, chained to upstream workspace or /opt/ros (or ``UNDERLAY``)
   4. run tests (opt-out with ``NOT_TEST_BUILD``)

C. Downstream workspace: Packages that should get tested against your target repository

   1. Fetch source code (``DOWNSTREAM_WORKSPACE``)
   2. Install dependencies with rosdep
   3. Build workspace ``$BASEDIR/${PREFIX}downstream_ws``, chained to target workspace
   4. run tests (opt-out with ``NOT_TEST_DOWNSTREAM``)

Workspace definition
++++++++++++++++++++

Each workspace can be composed as a sequence of the following items:

* URL of a source repository with the pattern ``<scheme>:<resource>#<version>``, e.g. ``github:ros-industrial/industrial_ci#master``.
  Supported scheme are:

  * ``github`` for GitHub repositories
  * ``gitlab`` for Gitlab repositories
  * ``bitbucket`` for Bitbucket repositories
  * ``git``/``git+*``: for any other git repository

  Please note that a version is mandatory. If you really want to use the default branch, which is error-prone and therefore not recommended, you can set it to ``HEAD``.

* URL (=starts with http or https) of a ``*.repos`` or ``*.rosinstall`` file
* relative path of a ``*.repos`` or ``*.rosinstall`` file
* (relative) directory path to a source directory
* directory path prefixed with ``-`` to remove the directory, as a path relative to either the source space or the target repository
* ``.`` to copy the full target repository

For backwards compatibility, ``UPSTREAM_WORKSPACE`` can be set to ``debian`` and ``file`` as well, but not in combination with the other options and with a deprecation warning.
In case of ``file``, it will be replaced by ``$ROSINSTALL_FILENAME`` or ``$ROSINSTALL_FILENAME.$ROS_DISTRO``, if the latter exists.
**In "file" mode the target repository will not get removed automatically anymore and therefore might get built twice!**

Examples:
+++++++++

To depend on a different GitHub repository, e.g. ros_control:
::

  UPSTREAM_WORKSPACE='github:ros-controls/ros_control#melodic-devel'


To depend on a different GitHub repository, e.g. ros_control, but only a subset of it:
::

  UPSTREAM_WORKSPACE='github:ros-controls/ros_control#melodic-devel -rqt_controller_manager'

**This does not remove the package, but the entire folder**

To depend on a remote rosinstall file instead, but still without ``rqt_controller_manager``:
::

  UPSTREAM_WORKSPACE='https://raw.githubusercontent.com/ros-controls/ros_control/melodic-devel/ros_control.rosinstall -ros_control/rqt_controller_manager'

Or to use a local copy:

::

  UPSTREAM_WORKSPACE='ros_control.rosinstall'

Works with (remote) ``*.repos`` as well:
::

  UPSTREAM_WORKSPACE='https://raw.githubusercontent.com/ros2/turtlebot2_demo/master/turtlebot2_demo.repos'

Or mixed:

::

  DOWNSTREAM_WORKSPACE="github:ros-simulation/gazebo_ros_pkgs#melodic-devel https://raw.githubusercontent.com/ros-controls/ros_control/melodic-devel/ros_control.rosinstall -ros_control additional.repos"

To depend on a different repository of a private server using git and the SSH protocol:
::

  UPSTREAM_WORKSPACE='git+ssh://git@private.server.net/repository#branch'

To filter the target workspace:
::

  TARGET_WORKSPACE='. -broken_package_path'

Use custom Docker images
------------------------

As you see in the `optional variables section <./index.rst#optional-environment-variables>`__, there are a few different ways to specify *Docker* image if you like. Here are some more detail:

Pulling Docker image from an online hub
+++++++++++++++++++++++++++++++++++++++

You can pull any *Docker* image by specifying in ``DOCKER_IMAGE`` variable.
If your *Docker* image is ROS-based, you can omit ``ROS_DISTRO`` as long as the Dockerfile sets this environment variable (``ENV ROS_DISTRO``)
However, ``ROS_REPO`` (or non-recommended ``ROS_REPOSITORY_PATH``), and ``ROS_DISTRO`` can still be used to modify the target container.

Please note that the entrypoint and command of the image will get ignored.

Pass custom variables to Docker
-------------------------------

On CI platform usually some variables are available for the convenience. Since all checks using ``industrial_ci`` are NOT running directly on the operating system running on CI, but instead running on *Docker* where those variables are not defined, dozens of them are already passed for you (you can see `the list of those variables <https://github.com/ros-industrial/industrial_ci/blob/master/industrial_ci/src/docker.env>`__).

Still, you may want to pass some other vars. ``DOCKER_RUN_OPTS='-e MY_VARIABLE_VALUE'`` should do the trick.
You can even set it to a specific value: ``DOCKER_RUN_OPTS='-e MY_VARIABLE_VALUE=42'`` (format varies per CI platform. These are Gitlab CI example).

Re-use the container image
--------------------------

NOTE: This is still experimental.

``industrial_ci`` builds a *Docker* image using the associated repository on the specified operating system per every job. While the built Docker container is thrown away once the job finishes by default, there's a way to access the built image post job so that you can re-use it.

To do so, simply set ``DOCKER_COMMIT`` the name of the image of your choice. Then you'll be able to access that image. For example in your CI config (e.g. ``.travis.yml``), add something like ::

  variables:
      DOCKER_COMMIT=registry.gitlab.com/your-org/your-repo:your_img
  :
  script:
      - docker push $DOCKER_COMMIT

(Gitlab CI) Access to private repositories
------------------------------------------

If your Gitlab CI jobs require access to private repos, additional settings are needed both on:

- Your repo: Add ssh private keys in the CI settings.
- The private repos the CI jobs access: Matching public keys must be set as ``Deploy Key``.

#. If you haven't done so, create SSH key pair (`reference on gitlab.com <https://docs.gitlab.com/ce/ssh/README.html#generating-a-new-ssh-key-pair>`__).
#. Navigate to "Settings > CI/CD" in your repo.
#. Expand "``Secret variables``" section.
#. In "Add a variable" section, fill in the following text field/area.

   #. **Key**: ``SSH_PRIVATE_KEY``
   #. **Value**: Copy paste the entire content of your private key file.

     #. Include the header and footer, i.e.  ``-----BEGIN/END RSA PRIVATE KEY-----``.
#. In "Add a variable" section again, fill in the following text field/area.

   #. **Key**: ``SSH_SERVER_HOSTKEYS``
   #. **Value**: Copy paste the entire line of the following: On your Linux computer, run ``ssh-keyscan gitlab.com``. You should get a hash key entry/ies. Copy the entire line that is NOT commented out. For example, the author gets the following, and copied the 2nd line (, which may render as separate lines on your web browser, but it's a long single line):

     ::

      # gitlab.com:22 SSH-2.0-OpenSSH_7.2p2 Ubuntu-4ubuntu2.2
      gitlab.com ssh-rsa RandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequence
      # gitlab.com:22 SSH-2.0-OpenSSH_7.2p2 Ubuntu-4ubuntu2.2
      gitlab.com ecdsa-sha2-nistp256 RandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequence
      # gitlab.com:22 SSH-2.0-OpenSSH_7.2p2 Ubuntu-4ubuntu2.2

#. Add a public key (reference for `Gitlab <https://docs.gitlab.com/ce/ssh/README.html#deploy-keys>`__ and for `GitHub <https://developer.github.com/v3/guides/managing-deploy-keys/#deploy-keys>`__) to the private repos your CI jobs accesses. You may need to ask the admin of that repo.
#. If you are using Docker-in-Docker, make sure that ``TMPDIR`` is set in your ``.gitlab-ci.yml`` file so that the SSH agent forwards properly ::

    # The docker runner does not expose /tmp to the docker-in-docker service
    # This config ensures that the temp folder is located inside the project directory (e.g. for prerelease tests or SSH agent forwarding)
    variables:
      TMPDIR: "${CI_PROJECT_DIR}.tmp"
#. If using a self-signed certificate you may need to make the container aware of the runner's certs ::

    kinetic:
      script:
        # Run the gitlab script, exposing the runner's SSL certs.
        - .industrial_ci/gitlab.sh DOCKER_RUN_OPTS="-v /etc/ssl/certs:/etc/ssl/certs:ro"


References:

- https://docs.gitlab.com/ce/ssh/README.html
- https://docs.gitlab.com/ee/ci/ssh_keys/README.html

(Recommended) Subscribe to the change in this repo (industrial_ci)
---------------------------------------------------------------------------------

Because of the aforementioned responsibility for the maintainers to watch the changes in ``industrial_ci``, `you're encouraged to subscribe to the updates in this repository <https://github.com/ros-industrial/industrial_ci/subscription>`__.

Run ROS Prerelease Test
-------------------------------------------------------------------------------------

Running `docker-based ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`__ is strongly recommended when you make a release. There are, however, some inconvenience (requires host computer setup, runs on your local host, etc. Detail discussed in `a ticket <https://github.com/ros-industrial/industrial_ci/pull/35#issue-150581346>`__). ``industrial_ci`` provides a way to run it on your CI.

To do so, add a single line to your `CI config <#terminology>`__:

::

  ROS_DISTRO=indigo PRERELEASE=true

Or with more configuration:

::

  ROS_DISTRO=indigo PRERELEASE=true PRERELEASE_REPONAME=industrial_core PRERELEASE_DOWNSTREAM_DEPTH=0

In addition to the downstream packages from ROS distro, you can specify ``UPSTREAM_WORKSPACE`` and ``DOWNSTREAM_WORKSPACE`` as well.

NOTE: A job that runs Prerelease Test does not run the checks that are defined in `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`__. To run both, use ``matrix`` in `CI config <#terminology>`__.

See the usage sample in `.travis in industrial_ci repository <https://github.com/ros-industrial/industrial_ci/blob/master/.travis.yml>`__.

The following is some tips to be shared for running Prerelease Test on CI using ``industrial_ci``.

(Workaround) Don't want to always run Prerelease Test
+++++++++++++++++++++++++++++++++++++++++++++++++++++

The jobs that run Prerelease Test may usually take longer than the tests defined in `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`__, which can result in longer time for the entire CI jobs to finish. This is usually okay, as developers who are concerned with PRs might not wait for the CI result that eagerly (besides that, most CI servers limit the maximum run time as 50 minutes so there can't be very long run). If you're concerned, however, then you may want to separately run the Prerelease Test. An example way to do this is to create a branch specifically for Prerelease Test where `CI config <#terminology>`__ only defines a check entry with ``PRERELEASE`` turned on. E.g.:

::

  :
  env:
    matrix:
      - ROS_DISTRO=indigo PRERELEASE=true
  :

Then open a pull request using this branch against the branch that the change is subject to be merged. You do not want to actually merge this branch no matter what the CI result is. This branch is solely for Prerelease Test purpose.

ABI checks
----------

Generally speaking, the `ABI <https://en.wikipedia.org/wiki/Application_binary_interface>`__ of a library can break for various reasons. A detailed explanation and a list of DOs and DON'Ts can be found in the `KDE Community Wiki <https://community.kde.org/Policies/Binary_Compatibility_Issues_With_C%2B%2B>`__.

The ABI checks with ``industrial_ci`` can be enabled by setting 'ABICHECK_URL' to the **stable version** of your code.

ABI check example configs
+++++++++++++++++++++++++

Simplest example: Check against a specific stable branch (e.g. ``kinetic`` branch) for push and pull request tests::

  - ROS_DISTRO=kinetic
    ABICHECK_URL='github:ros-industrial/ros_canopen#kinetic'

If pull requests should be checked against the merge parent instead of the stable version (Travis CI only). The only benefit is that PRs might pass even if the target branch breaks the ABI to the stable version.::

  - ROS_DISTRO=kinetic
    ABICHECK_URL='github:ros-industrial/ros_canopen#kinetic'
    ABICHECK_MERGE=auto

URL can be specified in shortcut form ``provider:organization/repository#version``, which is supported for bitbucket, github and gitlab. "``version``" can be either one of the name of the branch, the tagged version, or even a commit. Some (more) concrete examples:

- github:ros-industrial-release/ros_canopen-release#upstream
- gitlab:ipa-mdl/ci-example#master
- github:ros-planning/moveit#0.9.9

Alternatively you can use the following forms as URL.:

- https://github.com/ros-industrial/ros_canopen/archive/kinetic.zip
- https://github.com/ros-industrial-release/ros_canopen-release/archive/upstream.zip
- https://gitlab.com/ipa-mdl/ci-example/repository/master/archive.zip
- https://github.com/ros-planning/moveit/archive/0.9.9.tar.gz

With this format, the URL needs to point to an actual archive. E.g. on GitHub, URL for a branch's archive can be https://github.com/organization/repository/archive/branch.zip

Tips for ABI check feature
++++++++++++++++++++++++++

It is up to each repository's maintainer for which baseline code you check ABI against. Here are some recommendations per possible situation:

- Development branch and stable branch (i.e. mirroring the released code) are separately maintained --> checking against stable branch.
- No stable branch -->

  - Check against the stable tagged version.
  - Or you could check against the same branch. This way:

    - ABI check runs per every change/push into your branch, which is superfluous.
    - Reasonable for pull requests.

Cache build artifacts to speed up the subsequent builds (if any)
----------------------------------------------------------------

If ``CCACHE_DIR`` is set (not set by default), `ccache <https://en.wikipedia.org/wiki/Ccache>`__ gets enabled for your build to speed up the subsequent builds in the same job if anything.
Recommended value is ``$HOME/.ccache``, but any non-used directory works.

https://docs.travis-ci.com/user/caching/#Arbitrary-directories

 * Enable cache. How to do so depends on the CI system of your choice.

   On Travis CI, add as follows (`refrence <https://docs.travis-ci.com/user/caching/#Arbitrary-directories>`__)::

    cache:
      directories:
        - $HOME/.ccache  # can be any valid cache location


 * Define ``CCACHE_DIR`` variable. You can apply to all of your jobs by something like below::

    env:
      global:
        - CCACHE_DIR=$HOME/.ccache
      matrix:
       :

Or define ``CCACHE_DIR`` per job.

NOTE:
  * Beware, if you use `run_ci <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#id39>`__, the files will be owned by root!
  * Caching may not work for packages with "smaller" number of files (see also `this discussion <https://github.com/ros-industrial/industrial_ci/pull/182>`__).
  * With Gitlab CI, cache should always inside the project folder (`reference <https://docs.gitlab.com/ee/ci/yaml/README.html#cachepaths>`__)::

     variables:
       CCACHE_DIR: ${CI_PROJECT_DIR}/ccache

     cache:
       key: "${CI_JOB_NAME}"
       paths:
         - ccache

Run pre/post-process custom commands
-----------------------------------------

You may want to add custom steps prior/subsequent to the setup defined in ``industrial_ci``. Example usecases:

* A device driver package X in your repository or in your repository's dependency requires a prorietary library installed. This library is publicly available, but not via apt or any package management system and thus the only way you can install it is in a classic way (unzip, run installer etc.) (`More discussion <https://github.com/ros-industrial/industrial_ci/issues/14>`__).

* You want to run ``ros_lint`` (`this discussion <https://github.com/ros-industrial/industrial_ci/issues/58#issuecomment-223601916>`__ may be of your interest).

Customize within the CI process
++++++++++++++++++++++++++++++++

If what you want to customize is within the `CI process <#what-are-checked>`__, you can specify the script(s) in ``BEFORE_*`` and/or ``AFTER_*`` variables.
The variables can be set for all functions, using the upper-case name, e.g. to run a script before ``install_target_dependencies`` you can specify ``BEFORE_INSTALL_TARGET_DEPENDENCIES`` or ``AFTER_INSTALL_TARGET_DEPENDENCIES`` to be run afterrwards.
``BEFORE_INIT`` will be run before anything else, ``AFTER_SCRIPT`` can be used to specify as script to be run after all successful tests.

For example::

  env:
    global:
      - BEFORE_INIT='./your_custom_PREprocess.sh'
      - AFTER_SCRIPT='./your_custom_POSTprocess.sh'
  script:
    - .industrial_ci/ci.sh

Multiple commands can be passed, as in a general ``bash`` manner.::

    - BEFORE_INIT='ls /tmp/1 && ls /tmp/2 || ls /tmp/3'

Multiple commands are easier to be handled if they are put into a dedicated script::

    - BEFORE_INIT='./my_before_script.sh'

NOTE: In general the scripts are run as root in a Docker container. If you configure a different (base) Docker image, the user could be changed to non-root. But since we need to install packages the (base) image should set-up ``sudo`` for this user.

The hooks will get run without a ROS environment (``setup.bash``).
If you need this environment, you can use the ``rosenv`` helper.
Optionally, it takes a command to be executed.

Examples:

* ``AFTER_SETUP_UPSTREAM_WORKSPACE='rosenv && echo "$ROS_DISTRO'"``
* ``AFTER_SETUP_UPSTREAM_WORKSPACE='rosenv ./my_script.sh'``

Furthermore, these  hooks scripts are run in a sub-shell and cannot change the build environment.
If a dependency needs to extend the build environment, the `*_EMBED` script can be used::

    - AFTER_INIT='./your_custom_PREprocess.sh'
    - AFTER_INIT_EMBED='source /opt/dependency/prepare_environment.sh'

**rosenv cannot be used in \*_EMBED hooks!**

Per default all scripts are run with unset variables disabled in bash.
It is possible to opt-out for an individual command by prefixing it with `ici_with_unset_variables`.

Customize outside of the CI process
+++++++++++++++++++++++++++++++++++

As `explained in Docker's usage <#use-custom-docker-images>`__ section, `main CI processes of industrial_ci <#what-are-checked>`__ run on *Docker*. There may be situations where you want to run additional processes before or after the main pipeline. This could be particularly the case when you'd like to take advantage of CI's native resources (e.g. environment variables your CI platform defines) more easily.

You can add your own commands before/after the main processes as follows.

::

  script:
    - ./your_non-docker_before.sh  <-- Runs on CI server natively.
    - .industrial_ci/ci.sh             <-- Runs on Docker on CI server.
    - ./your_non-docker_after.sh   <-- Runs on CI server natively.

NOTE. CI native env vars can be sent to Docker (see `this section <#pass-custom-variables-to-docker>`__). The example above is useful e.g. when you have many variables to deal with. Anyways, both ways are valid.

Type of OS and distribution
--------------------------------------

Ubuntu and its distro are guessed by default from ROS_DISTRO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

You can specify the OS and its distribution to run the CI job by setting ``OS_NAME`` and ``OS_CODE_NAME``.
By default users don't need to set this and its value will be automatically guessed according to the value of ``ROS_DISTRO``. e.g.:

* ``ROS_DISTRO=indigo``  --> ``OS_NAME=ubuntu OS_CODE_NAME=trusty``
* ``ROS_DISTRO=kinetic`` --> ``OS_NAME=ubuntu OS_CODE_NAME=xenial``
* ``ROS_DISTRO=lunar``   --> ``OS_NAME=ubuntu OS_CODE_NAME=xenial``
* ``ROS_DISTRO=melodic`` --> ``OS_NAME=ubuntu OS_CODE_NAME=bionic``

Use non-default Ubuntu distro
+++++++++++++++++++++++++++++

E.g. ``OS_CODE_NAME=yakkety`` or ``zesty`` for ROS Lunar are available.

Use Debian
++++++++++

E.g.:

* ``OS_CODE_NAME=jessie``
* ``OS_CODE_NAME=stretch``

All combinations available of OS and distros
++++++++++++++++++++++++++++++++++++++++++++++

Possible combination of ``OS_NAME`` and ``OS_CODE_NAME`` depend on available Docker images. See `ros-industrial/docker/ci <https://github.com/ros-industrial/docker/tree/master/ci>`__.

Run industrial_ci on local host
---------------------------------------

There are a few ways to run CI jobs locally.

Simplest way to run locally
++++++++++++++++++++++++++++++++

Since version 0.3.3, you can run ``industrial_ci`` on your local host. This can be useful e.g. when you want to integrate industrial_ci into your CI server.

NOTE that this way the CI config (e.g. ``.travis.yml``, ``.gitlab-ci.yml``) are not used. So whatever configurations you have in your CI configs need to be added manually.

To do so,

0. `Install Docker <https://docs.docker.com/engine/installation/linux/>`__
1. Build and install industrial_ci (which is `a catkin package <http://wiki.ros.org/ROS/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.What_makes_up_a_catkin_Package.3F>`__). Source setting.
2. Change directory to the package you like to test.
3. Run ``run_ci`` script with your settings.

Example:

::

  $ cd ~/cws/src && git clone https://github.com/ros-industrial/industrial_ci.git -b master && cd ~/cws
  $ catkin config --install
  $ catkin b industrial_ci
  $ source install/setup.bash
  $ roscd ros_canopen   (or any package you test)
  $ rosrun industrial_ci run_ci ROS_DISTRO=indigo ROS_REPO=main

(ROS_DISTRO could be read from your environment as well)

Run locally using Travis config
++++++++++++++++++++++++++++++++

Since v0.6.0, you can run locally using ``.travis.yml`` you already defined for your repository, using `industrial_ci/scripts/run_travis script <https://github.com/ros-industrial/industrial_ci/blob/master/industrial_ci/scripts/run_travis>`_. See the help of that script.

::

   rosrun industrial_ci run_travis --help

Run locally using GitHub-Workflow config
++++++++++++++++++++++++++++++++++++++++

You can run GitHub actions locally using `act <https://github.com/nektos/act>`__.
For its installation, follow the `official install instructions <https://github.com/nektos/act#installation>`__.
Installation in short:

* install docker engine;
* install ``act`` using `bash script <https://github.com/nektos/act#bash-script>`__ (tested on Ubuntu and its derivatives) or download the `static binaries <https://github.com/nektos/act/releases>`__;
* When asked about which ``act`` image you would like to install, choose medium (default choice).

Before running a GH-Action locally, please check that you are using the `industrial_ci` as follows:

::

  - uses: ros-industrial/industrial_ci@master
    env:
      ROS_DISTRO: ${{ matrix.ROS_DISTRO }}
      ROS_REPO: ${{ matrix.ROS_REPO }}

Or for more complicated cases:

::

  - uses: ros-industrial/industrial_ci@master
      with:
        config: ${{toJSON(matrix.env)}}

Often used configuration is actually not supported by GH (more details in #590)

::

  - uses: ros-industrial/industrial_ci@master
      env: ${{matrix.env}}

After that, go to the package you would like to test and start a workflow using act:

* ``act`` - execute all workflows
* ``act -l`` - list all defined workflows
* ``act -j <my_workflow>`` - execute specific workflow

Some useful flags:

* to get more detailed output, use ``-v`` flag
* to reuse action containers, use ``-r`` flag (makes your actions much faster)
* for everything else check `act flags <https://github.com/nektos/act#flags>`__

Recurring runs for debugging
++++++++++++++++++++++++++++
Please note that ``run_ci`` and ``run_travis`` will download all dependencies every time, just as CI services would do.
For recurring runs, e.g. in a debugging session, this might not be desired.

As an alternative ``rerun_ci`` could be used. It take the same argument as ``run_ci`` (note for `some limitations <#note-for-rerun-ci-limitations>`__), but will run the build incrementally and only download or compile after changes.

This results in much faster execution for recurring runs, but has some disadvantages as well:

* The user needs to clean-up manually, an instruction to do so is printed at the end of all runs.
* All parameters incl. the repository path have to be passed explicitly to allow for proper caching.
* The apt dependencies won't get updated in recurring runs.
* Incremental builds might not work properly for all cases. Especially, it does not help with prerelease tests.

Example:

::

  $ rosrun industrial_ci rerun_ci . ROS_DISTRO=melodic ROS_REPO=main

This will run the tests and commit the result to a Docker image ``industrial-ci/rerun_ci/ros_canopen:$HASH``.
The hash is unique for each argument list, so ``rerun_ci . ROS_DISTRO=melodic`` and ``rerun_ci . ROS_DISTRO=kinetic`` do not mix  up.
However, it will keep consuming disk space with each new combination.

The cached images can be listed with
::

  $ rosrun industrial_ci rerun_ci --list

Note for rerun_ci limitations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``rerun_ci`` is managing ``DOCKER_COMMIT`` and ``DOCKER_COMMIT_MSG`` variables under the hood, so if the user set them they will not take effect, unlike `normal cases <#re-use-the-container-image>`__.

If you are using this feature to have a cached way to run ci locally you probably want your dependencies to be updated just as they are when run on a remote ci service.  To achieve this you can cause the target workspace to be pulled by adding this argument: ``AFTER_SETUP_TARGET_WORKSPACE='vcs pull ~/target_ws/src/'``.

For maintainers of industrial_ci repository
================================================

Checks for industrial_ci repo itself
---------------------------------------

While this repository provides CI scripts that can be used by other repositories, it also checks this repo itself using the same CI scripts and the simplest package setting. That is why this repo contains the ROS package files and a test (``CMakeLists.txt``, ``package.xml``).
