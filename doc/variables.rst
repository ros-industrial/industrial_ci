.. role:: raw-html(raw)
   :format: html

Reference of configuration variables
====================================

.. contents::

ROS_DISTRO
------------------------------------
Version of ROS in all lower case. E.g.: `indigo` / `jade`


ADDITIONAL_DEBS
------------------------------------
*(default: not set)*

More DEBs to be used. List the name of DEB(s delimitted by whitespace if multiple DEBs specified). Needs to be full-qualified Ubuntu package name. E.g.: "ros-indigo-roslint ros-indigo-gazebo-ros" (without quotation).


AFTER_SCRIPT
------------------------------------
*(default: not set)*

Used to specify shell commands that run after all source tests. NOTE: `Unlike Travis CI <https://docs.travis-ci.com/user/customizing-the-build#Breaking-the-Build>`_ where `after_script` doesn't affect the build result, the result in the commands specified with this DOES affect the build result.


BEFORE_SCRIPT
------------------------------------
*(default: not set)*

Used to specify shell commands that run before building packages.


BUILD_PKGS_WHITELIST
------------------------------------
*(default: not set)*

Packages to be built can be explicitly specified with this, in ROS package name format (i.e. using underscore. No hyphen). This is useful when your repo contains some packages that you don't want to be used upon testing. Downstream packages, if necessary, should be also specified using this. Also these packages are to be built when `NOT_TEST_INSTALL` is set. Finally, packages specified with this will be built together with those speicified using unimplmented `USE_DEB`.


BUILDER
------------------------------------
*(default: catkin)*

Currently only `catkin` is implemented (and with that `catkin_tools` is used instead of `catkin_make`. See `this discussion <https://github.com/ros-industrial/industrial_ci/issues/3>`_).


CATKIN_CONFIG
------------------------------------
*(default: not set)*

`catkin config --install` is used by default and with this variable you can 1) pass additional config options, or 2) overwrite `--install` by `--no-install`. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-customize-catkin-config>`_.


CATKIN_LINT
------------------------------------
*(default: not set. Value range: [true|pedantic])*

If `true`, run `catkin_lint <http://fkie.github.io/catkin_lint/>`_ with `--explain` option. If `pedantic`, `catkin_lint` command runs with `--strict -W2` option, i.e. more verbose output will print, and the CI job fails if there's any error and/or warning occurs.


CATKIN_LINT_ARGS
------------------------------------
*(default: not set)*

If true, you can pass whatever argument(s) `catkin_lint` takes, except `--explain` that is set by default. Options can be delimit by space if passing multiple.


CATKIN_PARALLEL_JOBS
------------------------------------
*(default: -p4)*

Maximum number of packages to be built in parallel that is passed to underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools`. See for more detail about `number of build jobs <http://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#controlling-the-number-of-build-jobs>`_ and `documentation of catkin_tools <https://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#full-command-line-interface>`_ that this env variable is passed to internally in `catkin-tools`.


CATKIN_PARALLEL_TEST_JOBS
------------------------------------
*(default: -p4)*

Maximum number of packages which could be examined in parallel during the test run. If not set it's filled by `ROS_PARALLEL_JOBS`.


CCACHE_DIR
------------------------------------
*(default: not set)*

If set, `ccache <https://en.wikipedia.org/wiki/Ccache>`_ gets enabled for your build to speed up the subsequent builds in the same job if anything. See `detail. <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#cache-build-artifacts-to-speed-up-the-subsequent-builds-if-any>`_


:raw-html:`<s>CI_PARENT_DIR</s>`
------------------------------------
**REMOVED**

*(default: .ci_config)*

(NOT recommended to specify) This is the folder name that is used in downstream repositories in order to point to this repo.


DEBUG_BASH
------------------------------------
*(default: not set)*

If set with any value (e.g. `true`), all executed commands that are not printed by default to reduce print space will be printed.


DOCKER_BASE_IMAGE
------------------------------------
*(default: $OS_NAME:$OS_CODE_NAME)*

Base image used for building the CI image. Could be used to pre-bundle dependecies or to run tests for different architectures. See `this PR <https://github.com/ros-industrial/industrial_ci/pull/174>`_ for more info.


DOCKER_IMAGE
------------------------------------
*(default: not set)*

Selects a Docker images different from default one. Please note, this disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO` as ROS needs already to be installed in the image.


DOCKER_FILE
------------------------------------
*(default: not set)*

Instead of pulling an images from the Docker hub, build it from the given path or URL. Please note, this disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO`, they have to be set in the build file instead.


DOCKER_BUILD_OPTS
------------------------------------
*(default: not set)*

Used do specify additional build options for Docker.


DOCKER_RUN_OPTS
------------------------------------
*(default: not set)*

Used do specify additional run options for Docker.


EXPECT_EXIT_CODE
------------------------------------
*(default: 0)*

exit code must match this value for test to succeed


NOT_TEST_BUILD
------------------------------------
*(default: not set)*

If true, tests in `build` space won't be run.


NOT_TEST_INSTALL
------------------------------------
*(default: not set)*

If true, tests in `install` space won't be run.


OS_CODE_NAME
------------------------------------
*(default: derived from ROS_DISTRO)*

See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`_.


OS_NAME
------------------------------------
*(default: ubuntu)*

Possible options: {`ubuntu`, `debian`}. See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`_.


PRERELEASE
------------------------------------
*(default: false)*

If `true`, run `Prerelease Test on docker that emulates ROS buildfarm <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`_. The usage of Prerelease Test feature is `explained more in this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-ros-prerelease-test>`_.


PRERELEASE_DOWNSTREAM_DEPTH
------------------------------------
*(0 to 4, default: 0)*

Number of the levels of the package dependecies the Prerelease Test targets at. Range of the level is defined by ROS buildfarm (`<http://prerelease.ros.org>`_). NOTE: a job can run exponentially longer for the values greater than `0` depending on how many packages depend on your package (and remember a job on Travis CI can only run for up to 50 minutes).


PRERELEASE_REPONAME
------------------------------------
*(default: TARGET_REPO_NAME)*

The  name of the target of Prerelease Test in rosdistro (that you select at `<http://prerelease.ros.org>`_). You can specify this if your repository name differs from the corresponding rosdisto entry. See `here <https://github.com/ros-industrial/industrial_ci/pull/145/files#r108062114>`_ for more usage.


PKGS_DOWNSTREAM
------------------------------------
*(default: explained)*

Packages in downstream to be tested. By default, `TARGET_PKGS` is used if set, if not then `BUILD_PKGS` is used.


ROS_PARALLEL_JOBS
------------------------------------
*(default: -j8)*

Maximum number of packages to be built in parallel by the underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).


ROS_PARALLEL_TEST_JOBS
------------------------------------
*(default: -j8)*

Maximum number of packages which could be examined in parallel during the test run by the underlining build tool. If not set it's filled by `ROS_PARALLEL_JOBS`. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).


ROS_REPO
------------------------------------
*(default: ros-shadow-fixed)*

`ROS_REPO` can be used to set `ROS_REPOSITORY_PATH` based on known aliases: 'ros`/`main`, 'ros-shadow-fixed`/`testing` or `building`.


ROS_REPOSITORY_PATH
------------------------------------
*(default: not set)*

Location of ROS' binary repositories where depended packages get installed from (typically both standard repo (`http://packages.ros.org/ros/ubuntu`) and `"Shadow-Fixed" repository <http://wiki.ros.org/ShadowRepository>`_ (`http://packages.ros.org/ros-shadow-fixed/ubuntu`)). Since version 0.3.4, `ROS_REPO` is recommended, and `ROS_REPOSITORY_PATH` is for more intermediate usage only (e.g. to specify your own binary repository (non-standard / in house)). Backward compatibility is preserved.


ROSDEP_SKIP_KEYS
------------------------------------
*(default: not set)*

space-separated list of keys that should get skipped by `rosdep install`.


ROSINSTALL_FILENAME
------------------------------------
*(default: .travis.rosinstall)*

Only used when `UPSTREAM_WORKSPACE` is set to `file`. See `UPSTREAM_WORKSPACE` description.


ROSWS
------------------------------------
*(default: wstool)*

Currently only `wstool` is available.


TARGET_PKGS
------------------------------------
*(default: not set)*

Used to fill `PKGS_DOWNSTREAM` if it is not set. If not set packages are set using the output of `catkin_topological_order` for the source space.


UPSTREAM_WORKSPACE
------------------------------------
*(default: debian)*

When set as `file`, the dependended packages that need to be built from source are downloaded based on a `.rosinstall` file in your repository. Use `$ROSINSTALL_FILENAME` to specify the file name. When set to a URL, downloads the rosinstall configuration from an ``http`` location. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#optional-build-depended-packages-from-source>`_.


USE_DEB
------------------------------------
**DEPRECATED**: use `UPSTREAM_WORKSPACE` instead.

*(default: true)*

if `true`, `UPSTREAM_WORKSPACE` will be set as `debian`. if `false`, `file` will be set. See `UPSTREAM_WORKSPACE` section for more info.


USE_MOCKUP
------------------------------------
*(default: not set)*

reletive path to mockup packages to be used for the tests


VERBOSE_OUTPUT
------------------------------------
*(default: not set)*

If `true`, build tool (e.g. Catkin) output prints in verbose mode.
