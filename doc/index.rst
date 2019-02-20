================
Industrial CI
================
Continuous integration repository for ROS-Industrial

.. contents:: Table of Contents
   :depth: 3

Introduction
============

This repository contains `CI (Continuous Integration) <https://en.wikipedia.org/wiki/Continuous_integration>`__ configuration that can be commonly used by the repositories in `ros-industrial <https://github.com/ros-industrial>`__ organization. Non ros-industrial repositories in other organizations can utilize the CI config here too, as long as they are ROS-powered.

As of November 2017, this repo provides configuration for `Travis CI` and `Gitlab CI`. The CI scripts in this repository are intended to be obtained by `git clone` feature. In client repos you can define custom, repository-specific checks, in addition to the generic configs stored in this repo.

For a brief introduction, you could also check a presentation:

* `ROS-Industrial community meeting <http://rosindustrial.org/news/2016/6/14/ros-i-community-web-meeting-june-2016>`__

Supported Platform
------------------

Supported ROS Distributions
+++++++++++++++++++++++++++

Following `ROS distributions <http://wiki.ros.org/action/login/Distributions>`__ are supported.

* `Hydro <http://wiki.ros.org/hydro>`__ *(EOL)*
* `Indigo <http://wiki.ros.org/indigo>`__ *(EOL)*
* `Jade <http://wiki.ros.org/jade>`__ *(EOL)*
* `Kinetic <http://wiki.ros.org/kinetic>`__
* `Lunar <http://wiki.ros.org/lunar>`__ *(EOL)*
* `Melodic <http://wiki.ros.org/melodic>`__

Supported CIs
+++++++++++++

* Gitlab CI
* Travis CI

As of January 2018, this document uses the format of `Travis CI` by default, unless specified.

Variety of operating system by utilizing Docker
-----------------------------------------------

After `version 0.3.3 <http://docs.ros.org/kinetic/changelogs/industrial_ci/changelog.html>`__, all checks run on `Docker` so that you can have the variety of the operating system to check your software against, freed from the limitation of your CI platform (e.g. as of 2017 on `Travis CI`, Ubuntu 16.04 isn't available yet).

Terminology
----------------

* **CI config**: `.travis.yml` for Travis CI. `.gitlab-ci.yml` for `Gitlab CI`.
* **client repository**: The repositories that use the configuration stored in this repo to run CI jobs.
* **downstream packages**: The software packages that depend on the package that's targetted to be tested using industrial_ci.
* **merge parent**: The branch that your pull/merge request is opened against.

FAQ
======

- Q- This config can be used ONLY by the repositories under `github/ros-industrial <https://github.com/ros-industrial>`__ organization?

  A- No. `industrial_ci` repo is open to public. Anyone can use this from any platform. Note that because as of Dec. 2015 it has only config for `Travis CI <https://travis-ci.org/>`__, you may want to use it where Travis CI is available (`github.com` works the best.

- Q- What kind of checks are implemented that are specific to industrial robotics?

  A- As of Dec. 2015, no particular configuration for industrial robot is defined.

- Q- So, can the config be used against any robotics repository?

  A- I'd say no. It's still limited for the projects based on `ROS <http://ros.org/>`__. And checks are run on Ubuntu linux.

- Q- In my project there aren't yet test cases. Can I still have it checked using `industrial_ci` and what can I get out of the check?

  A- The `industrial_ci` still provides valuable checks; it ensures if your package builds without issues. Also installation rules if you define. Just as a headsup that making test cases are highly recommended as your ear may hurt.

- Q- My package uses a custom Point Cloud Library (PCL) version or the `industrial_calibration <https://github.com/ros-industrial/industrial_calibration>`__ package, how do I make build work?

  A- You can check `advanced_industrial_ci <https://github.com/InstitutMaupertuis/advanced_industrial_ci>`__ which provides scripts and binaries for PCL 1.8.0 and the Ceres solver. An integration example can be found `here <https://github.com/InstitutMaupertuis/ensenso_extrinsic_calibration/blob/indigo-devel/.travis.yml>`__.

- Q- How does the target package get installed?

  A- `Travis CI` does this. It pulls in your package to an running instance of an operating system of your choice, and place your package under `/home/travis`.

- Q- The jobs on `Travis CI` are failing. How can I fix them?

  A- (1) Find the section where error occurred that caused CI to stop. Sections are folded nicely and it's colored red when it fails. (2) Then identify whether the issue is pertaining to your package, or something else. Sometimes a cause is in `industrial_ci`, not your package. (3) Reviewing `Common Build Problems for Travis CI <https://docs.travis-ci.com/user/common-build-problems>`__ helps you to isolate the root cause. (4) If you think the root cause is in `industrial_ci`, (or if you're not sure,) ask at its `issue tracker <https://github.com/ros-industrial/industrial_ci/issues>`__.

- Q- How can I customize the jobs?

  A- (1) There are a number of variables to customize your jobs that you can learn the usage `in this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#variables-you-can-configure>`__. (2) You can define pre- and post-processes, in addition to the default scripts (it's `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`__ for `Travis CI`). See `this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-pre-post-process-custom-commands>`__ for how.

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

To run `industrial_ci`, each package in your repository needs to be:

* compatible on the `Supported Platform <#supported-platform>`__.
* `"Catkin package" <http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage>`__ (uses CMake for build configuration), since many checks are triggered by the `Catkin`-based commands.

Basic Usage
===========

Here are some operations in your client repositories.

To start using CI config stored in this repo
--------------------------------------------------

With the following few short steps, you can start in your client repository using `industrial_ci` scripts.

1. Don't forget to activate CI for your repository.

   * For Travis CI and GitHub, you may do so on https://travis-ci.org/profile/YOUR_GITHUB_ORGANIZATION or https://travis-ci.org/profile/YOUR_GITHUB_USER (replace capital with your value).
2. In `CI config <#terminology>`__ file in your client repo include and run industrial_ci.

   * For Travis create the file `.travis.yml <https://github.com/ros-industrial/industrial_ci/blob/master/doc/.travis.yml>`__.
   * A Gitlab CI config can be found in `.gitlab-ci.yml <https://github.com/ros-industrial/industrial_ci/blob/master/.gitlab-ci.yml>`__.

That's it.

Apply the changes in this repo (industrial_ci) to the checking in client repos
----------------------------------------------------------------------------------

Nothing.
Once you add `git clone` statement in your client repo, basically you don't need to do anything to apply the change in `industrial_ci` repository.

Example client packages
-------------------------------

* `ros-industrial/industrial_core <https://github.com/ros-industrial/industrial_core/blob/indigo-devel/.travis.yml>`__
* `ros-industrial-consortium/descartes <https://github.com/ros-industrial-consortium/descartes/blob/indigo-devel/.travis.yml>`__

Advanced Usage
==============

Variables you can configure
------------------------------------

You can configure the behavior in `CI config <#terminology>`__ in your client repository.

* OS to use. Defined at `dist` tag.

Required environment variables:

* `ROS_DISTRO`: Version of ROS in all lower case. E.g.: `indigo`. If is is set in the custom Docker (base) image, it might be omitted in the script call.

Optional environment variables
++++++++++++++++++++++++++++++++

Note that some of these currently tied only to a single option, but we still leave them for the future when more options become available (e.g. ament with BUILDER).

* **ABICHECK_MERGE** (default: not set): Used only when `ABICHECK_URL` is set. For travis it can be set to 'auto' to auto-detect pull requests. If set to 'true' the merge parent (see `Terminology section <#terminology>`__) will be checked against.
* **ABICHECK_URL** (default: not set): Run binary compatibility check with `ABICC <https://github.com/lvc/abi-compliance-checker>`__. The URL should point to a baseline archive (\*.tar.\*,\*.zip, \*.tgz or \*.tbz2). See more in `the ABI checks section <#abi-checks>`__)
* **ABICHECK_VERSION** (default: not set): Used only when `ABICHECK_URL` is set. Version name (for display only) of the set of code, which the location is specified in `ABICHECK_URL` of. The version will be automatically read from the URL passed in `ABICHECK_URL` if possible, but for a URL that doesn't point to a version-based file name (e.g. the link for a tagged version on Gitlab doesn't).
* **ADDITIONAL_DEBS** (default: not set): More DEBs to be used. List the name of DEB(s delimitted by whitespace if multiple DEBs specified). Needs to be full-qualified Ubuntu package name. E.g.: "ros-indigo-roslint ros-indigo-gazebo-ros" (without quotation).
* **AFTER_SCRIPT**: (default: not set): Used to specify shell commands that run after all source tests. NOTE: `Unlike Travis CI <https://docs.travis-ci.com/user/customizing-the-build#Breaking-the-Build>`__ where `after_script` doesn't affect the build result, the result in the commands specified with this DOES affect the build result. See more `here <./index.rst#run-pre-post-process-custom-commands>`__.
* **BEFORE_SCRIPT**: (default: not set): Used to specify shell commands that run before building packages (more precisely, it gets called after the workspace to be built is prepared, but before the dependency of packages in that workspace is resolved). See more `here <./index.rst#run-pre-post-process-custom-commands>`__.
* **BUILD_PKGS_WHITELIST** (default: not set): Packages to be built can be explicitly specified with this, in ROS package name format (i.e. using underscore. No hyphen). This is useful when your repo contains some packages that you don't want to be used upon testing. Downstream packages, if necessary, should be also specified using this. Also these packages are to be built when `NOT_TEST_INSTALL` is set. Finally, packages specified with this will be built together with those speicified using unimplmented `USE_DEB`.
* **BUILDER** (default: catkin): Currently only `catkin` is implemented (and with that `catkin_tools` is used instead of `catkin_make`. See `this discussion <https://github.com/ros-industrial/industrial_ci/issues/3>`__).
* **CATKIN_CONFIG** (default: not set): `catkin config --install` is used by default and with this variable you can 1) pass additional config options, or 2) overwrite `--install` by `--no-install`. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-customize-catkin-config>`__.
* **CATKIN_LINT** (default: not set. Value range: [true|pedantic]): If `true`, run `catkin_lint <http://fkie.github.io/catkin_lint/>`__ with `--explain` option. If `pedantic`, `catkin_lint` command runs with `--strict -W2` option, i.e. more verbose output will print, and the CI job fails if there's any error and/or warning occurs.
* **CATKIN_LINT_ARGS** (default: not set): If true, you can pass whatever argument(s) `catkin_lint` takes, except `--explain` that is set by default. Options can be delimit by space if passing multiple.
* **CATKIN_PARALLEL_JOBS** (default: -p4): Maximum number of packages to be built in parallel that is passed to underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools`. See for more detail about `number of build jobs <http://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#controlling-the-number-of-build-jobs>`__ and `documentation of catkin_tools <https://catkin-tools.readthedocs.org/en/latest/verbs/catkin_build.html#full-command-line-interface>`__ that this env variable is passed to internally in `catkin-tools`.
* **CATKIN_PARALLEL_TEST_JOBS** (default: -p4): Maximum number of packages which could be examined in parallel during the test run. If not set it's filled by `ROS_PARALLEL_JOBS`.
* **CMAKE_ARGS** (default: not set): CMake arguments that get passed to the builder.
* **CCACHE_DIR** (default: not set): If set, `ccache <https://en.wikipedia.org/wiki/Ccache>`__ gets enabled for your build to speed up the subsequent builds in the same job if anything. See `detail. <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#cache-build-artifacts-to-speed-up-the-subsequent-builds-if-any>`__
* **CLANG_FORMAT_CHECK** (default: not set. Value range: ``[<format-style>|file]``): If set, run the `clang-format <https://clang.llvm.org/docs/ClangFormat.html>`__ check. Set the argument to ``file`` if the style configuration should be loaded from a ``.clang-format`` file, located in one of the parent directories of the source file.
* **CLANG_FORMAT_VERSION** (default: not set): Version of clang-format to install and use (relates to both the apt package name as well as the executable), e.g., ``CLANG_FORMAT_VERSION=3.8``.
* **DEBUG_BASH** (default: not set): If set with any value (e.g. `true`), all executed commands that are not printed by default to reduce print space will be printed.
* **DOCKER_BASE_IMAGE** (default: $OS_NAME:$OS_CODE_NAME): Base image used for building the CI image. Could be used to pre-bundle dependecies or to run tests for different architectures. See `this PR <https://github.com/ros-industrial/industrial_ci/pull/174>`__ for more info.
* **DOCKER_BUILD_OPTS** (default: not set): Used do specify additional build options for Docker.
* **DOCKER_COMMIT** (default: not set): If set, the docker image, which contains the build and test artifacts, will be saved in the outer-layer docker which runs the ``industrial_ci`` script and thus will become accessible for later usage (e.g. you can then push to your docker registry). If unset, the container will not be commited and is removed. The value is used to specify an image name during the ``docker commit`` command.
* **DOCKER_COMMIT_MSG** (default: not set): used to specify a commit during the docker commit command which is triggered by setting ``DOCKER_COMMIT``. If unset and if ``DOCKER_COMMIT`` is set then the commit message will be empty. See more ``DOCKER_COMMIT``.
* **DOCKER_FILE** (default: not set): Instead of pulling an images from the Docker hub, build it from the given path or URL. Please note, this disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO`, they have to be set in the build file instead.
* **DOCKER_IMAGE** (default: not set): Selects a Docker images different from default one. Please note, this disables the handling of `ROS_REPOSITORY_PATH` and `ROS_DISTRO` as ROS needs already to be installed in the image.
* **DOCKER_PULL** (default: true): set to false if custom docker image should not be pulled, e.g. if it was created locally
* **DOCKER_RUN_OPTS** (default: not set): Used to specify additional run options for Docker.
* **EXPECT_EXIT_CODE** (default: 0): exit code must match this value for test to succeed
* **INJECT_QEMU** (default: not set): Inject static qemu emulator for cross-platform builds, e.g. `INJECT_QEMU=arm`. This requires to install `qemu-user-static` on the host. The emulated build might take much longer!
* **NOT_TEST_BUILD** (default: not set): If true, tests in `build` space won't be run.
* **OS_CODE_NAME** (default: derived from ROS_DISTRO): See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`__.
* **OS_NAME** (default: ubuntu): Possible options: {`ubuntu`, `debian`}. See `this section for the detail <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#optional-type-of-os-and-distribution>`__.
* **PRERELEASE** (default: false): If `true`, run `Prerelease Test on docker that emulates ROS buildfarm <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`__. The usage of Prerelease Test feature is `explained more in this section <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#run-ros-prerelease-test>`__.
* **PRERELEASE_DOWNSTREAM_DEPTH** (0 to 4, default: 0): Number of the levels of the package dependecies the Prerelease Test targets at. Range of the level is defined by ROS buildfarm (`<http://prerelease.ros.org>`__). NOTE: a job can run exponentially longer for the values greater than `0` depending on how many packages depend on your package (and remember a job on Travis CI can only run for up to 50 minutes).
* **PRERELEASE_REPONAME** (default: TARGET_REPO_NAME): The  name of the target of Prerelease Test in rosdistro (that you select at `<http://prerelease.ros.org>`__). You can specify this if your repository name differs from the corresponding rosdisto entry. See `here <https://github.com/ros-industrial/industrial_ci/pull/145/files#r108062114>`__ for more usage.
* **PKGS_DOWNSTREAM** (default: explained): Packages in downstream to be tested. By default, `TARGET_PKGS` is used if set, if not then `BUILD_PKGS_WHITELIST` is used.
* **ROS_PARALLEL_JOBS** (default: -j8): Maximum number of packages to be built in parallel by the underlining build tool. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).
* **ROS_PARALLEL_TEST_JOBS** (default: -j8): Maximum number of packages which could be examined in parallel during the test run by the underlining build tool. If not set it's filled by `ROS_PARALLEL_JOBS`. As of Jan 2016, this is only enabled with `catkin_tools` (with `make` as an underlining builder).
* **ROS_REPO** (default: ros-shadow-fixed): `ROS_REPO` can be used to set `ROS_REPOSITORY_PATH` based on known aliases: 'ros`/`main`, 'ros-shadow-fixed`/`testing` or `building`.
* **ROS_REPOSITORY_PATH**: Location of ROS' binary repositories where depended packages get installed from (typically both standard repo (`http://packages.ros.org/ros/ubuntu`) and `"Shadow-Fixed" repository <http://wiki.ros.org/ShadowRepository>`__ (`http://packages.ros.org/ros-shadow-fixed/ubuntu`)). Since version 0.3.4, `ROS_REPO` is recommended, and `ROS_REPOSITORY_PATH` is for more intermediate usage only (e.g. to specify your own binary repository (non-standard / in house)). Backward compatibility is preserved.
* **ROSDEP_SKIP_KEYS** (default: not set): space-separated list of keys that should get skipped by `rosdep install`.
* **ROSINSTALL_FILENAME** (default: .travis.rosinstall): Only used when `UPSTREAM_WORKSPACE` is set to `file`. See `UPSTREAM_WORKSPACE` description.
* **TARGET_PKGS** (default: not set): Used to fill `PKGS_DOWNSTREAM` if it is not set. If not set packages are set using the output of `catkin_topological_order` for the source space.
* **UPSTREAM_WORKSPACE** (default: debian): When set as `file`, the dependended packages that need to be built from source are downloaded based on a `.rosinstall` file in your repository. Use `$ROSINSTALL_FILENAME` to specify the file name. When set to a URL, downloads the rosinstall configuration from an ``http`` location. See more in `this section <https://github.com/ros-industrial/industrial_ci/blob/master/README.rst#optional-build-depended-packages-from-source>`__.
* **USE_DEB** (*DEPRECATED*: use `UPSTREAM_WORKSPACE` instead. default: true): if `true`, `UPSTREAM_WORKSPACE` will be set as `debian`. if `false`, `file` will be set. See `UPSTREAM_WORKSPACE` section for more info.
* **USE_MOCKUP** (default: not set): reletive path to mockup packages to be used for the tests
* **VERBOSE_OUTPUT** (default: not set): If `true`, build tool (e.g. Catkin) output prints in verbose mode.
* **VERBOSE_TESTS** (default: true): If `true`, build tool (e.g. Catkin) output prints in verbose mode during `run_tests` step.

Note: You see some `*PKGS*` variables. These make things very flexible but in normal usecases you don't need to be bothered with them - just keep them blank.

Use custom Docker images
------------------------

As you see in the `optional variables section <./index.rst#optional-environment-variables>`__, there are a few different ways to specify `Docker` image if you like. Here are some more detail:

Pulling Docker image from an online hub
+++++++++++++++++++++++++++++++++++++++

You can pull any `Docker` image by specifying in `DOCKER_IMAGE` variable, as long as the following requirement is met:

* sources.list set up (`example <http://wiki.ros.org/kinetic/Installation/Ubuntu#Installation.2BAC8-Ubuntu.2BAC8-Sources.Setup_your_sources.list>`__).
* `python-catkin-tools`, `python-pip`, `python-rosdep`, `python-wstool`. If your Docker image is missing any of the above libraries, then you can still pass their name by `ADDITIONAL_DEBS` (see `variables section <./index.rst#optional-environment-variables>`__).

Some more notes:

* Setting `DOCKER_IMAGE` is a bit tricky:
   * disables the set-up of ROS based on `ROS_REPO` (or non-recommended `ROS_REPOSITORY_PATH`), and ROS_DISTRO.
   * but `ROS_DISTRO` needs to be set if it was not set in the image.
* Some common credentials such as `.docker`, `.ssh`, `.subversion` are passed from CI native platform to Docker container.

Pass custom variables to Docker
-------------------------------

On CI platform usually some variables are available for the convenience. Since all checks using `industrial_ci` are NOT running directly on the operating system running on CI, but instead running on `Docker` where those variables are not defined, dozens of them are already passed for you (you can see `the list of those variables <https://github.com/ros-industrial/industrial_ci/blob/master/industrial_ci/src/docker.env>`__).

Still, you may want to pass some other vars. `DOCKER_RUN_OPTS='-e MY_VARIABLE_VALUE'` should do the trick.
You can even set it to a specific value: `DOCKER_RUN_OPTS='-e MY_VARIABLE_VALUE=42'` (format varies per CI platform. These are Gitlab CI example).

Re-use the container image
--------------------------

NOTE: This is still experimental.

``industrial_ci`` builds a ``Docker`` image using the associated repository on the specified operating system per every job. While the built Docker container is thrown away once the job finishes by default, there's a way to access the built image post job so that you can re-use it.

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
- The private repos the CI jobs access: Matching public keys must be set as `Deploy Key`.

#. If you haven't done so, create SSH key pair (`reference on gitlab.com <https://docs.gitlab.com/ce/ssh/README.html#generating-a-new-ssh-key-pair>`__).
#. Navigate to "Settings > CI/CD" in your repo.
#. Expand "`Secret variables`" section.
#. In "Add a variable" section, fill in the following text field/area.

   #. **Key**: `SSH_PRIVATE_KEY`
   #. **Value**: Copy paste the entire content of your private key file.

     #. Include the header and footer, i.e.  `-----BEGIN/END RSA PRIVATE KEY-----`.
#. In "Add a variable" section again, fill in the following text field/area.

   #. **Key**: `SSH_SERVER_HOSTKEYS`
   #. **Value**: Copy paste the entire line of the following: On your Linux computer, run `ssh-keyscan gitlab.com`. You should get a hash key entry/ies. Copy the entire line that is NOT commented out. For example, the author gets the following, and copied the 2nd line (, which may render as separate lines on your web browser, but it's a long single line):

     ::

      # gitlab.com:22 SSH-2.0-OpenSSH_7.2p2 Ubuntu-4ubuntu2.2
      gitlab.com ssh-rsa RandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequence
      # gitlab.com:22 SSH-2.0-OpenSSH_7.2p2 Ubuntu-4ubuntu2.2
      gitlab.com ecdsa-sha2-nistp256 RandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequenceRandomKeySequence
      # gitlab.com:22 SSH-2.0-OpenSSH_7.2p2 Ubuntu-4ubuntu2.2

#. Add a public key (reference for `Gitlab <https://docs.gitlab.com/ce/ssh/README.html#deploy-keys>`__ and for `GitHub <https://developer.github.com/v3/guides/managing-deploy-keys/#deploy-keys>`__) to the private repos your CI jobs accesses. You may need to ask the admin of that repo.

References:

- https://docs.gitlab.com/ce/ssh/README.html
- https://docs.gitlab.com/ee/ci/ssh_keys/README.html

(Recommended) Subscribe to the change in this repo (industrial_ci)
---------------------------------------------------------------------------------

Because of the aforementioned responsibility for the maintainers to watch the changes in `industrial_ci`, `you're encouraged to subscribe to the updates in this repository <https://github.com/ros-industrial/industrial_ci/subscription>`__.

Run ROS Prerelease Test
-------------------------------------------------------------------------------------

Running `docker-based ROS Prerelease Test <http://wiki.ros.org/bloom/Tutorials/PrereleaseTest/>`__ is strongly recommended when you make a release. There are, however, some inconvenience (requires host computer setup, runs on your local host, etc. Detail discussed in `a ticket <https://github.com/ros-industrial/industrial_ci/pull/35#issue-150581346>`__). `industrial_ci` provides a way to run it on your CI.

To do so, add a single line to your `CI config <#terminology>`__:

::

  ROS_DISTRO=indigo PRERELEASE=true

Or with more configuration:

::

  ROS_DISTRO=indigo PRERELEASE=true PRERELEASE_REPONAME=industrial_core PRERELEASE_DOWNSTREAM_DEPTH=0

NOTE: A job that runs Prerelease Test does not run the checks that are defined in `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`__. To run both, use `matrix` in `CI config <#terminology>`__.

See the usage sample in `.travis in indusrial_ci repository <https://github.com/ros-industrial/industrial_ci/blob/master/.travis.yml>`__.

The following is some tips to be shared for running Prerelease Test on CI using `industrial_ci`.

(Workaround) Don't want to always run Prerelease Test
+++++++++++++++++++++++++++++++++++++++++++++++++++++

The jobs that run Prerelease Test may usually take longer than the tests defined in `travis.sh <https://github.com/ros-industrial/industrial_ci/blob/master/travis.sh>`__, which can result in longer time for the entire CI jobs to finish. This is usually okay, as developers who are concerned with PRs might not wait for the CI result that eagerly (besides that, most CI servers limit the maximum run time as 50 minutes so there can't be very long run). If you're concerned, however, then you may want to separately run the Prerelease Test. An example way to do this is to create a branch specifically for Prerelease Test where `CI config <#terminology>`__ only defines a check entry with `PRERELEASE` turned on. E.g.:

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

The ABI checks with `industrial_ci` can be enabled by setting 'ABICHECK_URL' to the **stable version** of your code.

ABI check example configs
+++++++++++++++++++++++++

Simplest example: Check against a specific stable branch (e.g. `kinetic` branch) for push and pull request tests::

  - ROS_DISTRO=kinetic
    ABICHECK_URL='github:ros-industrial/ros_canopen#kinetic'

If pull requests should be checked against the merge parent instead of the stable version (Travis CI only). The only benefit is that PRs might pass even if the target branch breaks the ABI to the stable version.::

  - ROS_DISTRO=kinetic
    ABICHECK_URL='github:ros-industrial/ros_canopen#kinetic'
    ABICHECK_MERGE=auto

URL can be specified in shortcut form `provider:organization/repository#version`, which is supported for bitbucket, github and gitlab. "`version`" can be either one of the name of the branch, the tagged version, or even a commit. Some (more) concrete examples:

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

Customize `catkin config`
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

 * `Discussion about install space <https://github.com/ros-industrial/industrial_ci/issues/54>`__
 * `Detail for catkin config <http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_config.html>`__ for more info about `catkin-tools`.

Cache build artifacts to speed up the subsequent builds (if any)
----------------------------------------------------------------

If `CCACHE_DIR` is set (not set by default), `ccache <https://en.wikipedia.org/wiki/Ccache>`__ gets enabled for your build to speed up the subsequent builds in the same job if anything.
Recommended value is `$HOME/.ccache`, but any non-used directory works.

https://docs.travis-ci.com/user/caching/#Arbitrary-directories

 * Enable cache. How to do so depends on the CI system of your choice.

   On Travis CI, add as follows (`refrence <https://docs.travis-ci.com/user/caching/#Arbitrary-directories>`__)::

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
  * Beware, if you use `run_ci <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#id39>`__, the files will be owned by root!
  * Caching may not work for packages with "smaller" number of files (see also `this discussion <https://github.com/ros-industrial/industrial_ci/pull/182>`__).

To use specific version of industrial_ci in your client repo
-------------------------------------------------------------------------------------

(A minor) downside of how you associate your client repo to this `industrial_ci` repository is that you have no control over which version to use (see `discussion in this ticket <https://github.com/ros-industrial/industrial_ci/issues/3>`__). If you wish you can specify the version.

The following is an example using `git submodule`. Note that when using this method, you have to manually update the `submodule` every time there's an update in this `industrial_ci` package.

First time you define the dependency to this repo
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

1. Run git submodule command.

::

  CLIENTREPO_LOCAL$ git submodule add https://github.com/ros-industrial/industrial_ci .industrial_ci

This standard `git submodule` command:

* hooks up your client repository to this repo by the name "`.industrial_ci`" (this name is hardcoded and mandatory).
* stores the configuration in a file called `.gitmodules`.

2. Don't forget to activate CI on your github repository (you may do so on https://travis-ci.org/profile/YOUR_GITHUB_USER).

3. In `CI config <#terminology>`__ file in your client repo, add the portion below:

::

  script:
    - .industrial_ci/ci.sh
    #- ./your_non-docker_after.sh  # Optional. Explained later

Also, the example of entire file `CI config <#terminology>`__ can be found in `industrial_core/.travis.yml <https://github.com/ros-industrial/industrial_core/.travis.yml>`__.

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

You may want to add custom steps prior/subsequent to the setup defined in `industrial_ci`. Example usecases:

* A device driver package X in your repository or in your repository's dependency requires a prorietary library installed. This library is publicly available, but not via apt or any package management system and thus the only way you can install it is in a classic way (unzip, run installer etc.) (`More discussion <https://github.com/ros-industrial/industrial_ci/issues/14>`__).

* You want to run `ros_lint` (`thi discussion <https://github.com/ros-industrial/industrial_ci/issues/58#issuecomment-223601916>`__ may be of your interest).

Customize within the CI process
++++++++++++++++++++++++++++++++

If what you want to customize is within the `CI process <#what-are-checked>`__, you can specify the script(s) in `BEFORE_SCRIPT` and/or `AFTER_SCRIPT` variables. For example::

  env:
    global:
      - BEFORE_SCRIPT='./your_custom_PREprocess.sh'
      - AFTER_SCRIPT='./your_custom_POSTprocess.sh'
  script:
    - .industrial_ci/ci.sh

Multiple commands can be passed, as in a general `bash` manner.::

    - BEFORE_SCRIPT='ls /tmp/1 && ls /tmp/2 || ls /tmp/3'

Multiple commands are easier to be handled if they are put into a dedicated script::

    - BEFORE_SCRIPT='./my_before_script.sh'

NOTE: In general the scripts are run as root in a Docker container. If you configure a different (base) Docker image, the user could be changed to non-root. But since we need to install packages the (base) image should set-up `sudo` for this user.

Customize outside of the CI process
+++++++++++++++++++++++++++++++++++

As `explained in Docker's usage <#use-custom-docker-images>`__ section, `main CI processes of industrial_ci <#what-are-checked>`__ run on `Docker`. There may be situations where you want to run additional processes before or after the main pipeline. This could be particularly the case when you'd like to take advantage of CI's native resources (e.g. environment variables your CI platform defines) more easily.

You can add your own commands before/after the main processes as follows.

::

  script:
    - ./your_non-docker_before.sh  <-- Runs on CI server natively.
    - .industrial_ci/ci.sh             <-- Runs on Docker on CI server.
    - ./your_non-docker_after.sh   <-- Runs on CI server natively.

NOTE. CI native env vars can be sent to Docker (see `this section <#pass-custom-variables-to-docker>`__). The example above is useful e.g. when you have many variables to deal with. Anyways, both ways are valid.

Build depended packages from source
----------------------------------------------

By default the packages your package depend upon are installed via binaries. However, you may want to build them via source in some cases (e.g. when depended binaries are not available). There are a few ways to do so in `industrial_ci`; By utilizing `rosinstall <http://docs.ros.org/independent/api/rosinstall/html/>`__, you can specify the packages that you want to be built from source.

Note that while building the designated packages from source, other packages are resolved still from binary automatically by `rosdep <http://wiki.ros.org/rosdep>`__.

Examples of how to enable all of the following cases are available in `.travis.yml file on this repository <https://github.com/ros-industrial/industrial_ci/blob/master/.travis.yml>`__.

Use .rosinstall file to specify the depended packages source repository
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

WARNING: In all cases where you want to utilize `.rosinstall` (or similar name) files, be sure to set `USE_DEB` as `false`, or simply not define it.

For using a rosinstall file located locally within the repository, define one or two variables as:

1) set `UPSTREAM_WORKSPACE` as `file`.
2) optionally create a file `$ROSINSTALL_FILENAME` using the same file format as `.rosinstall <http://docs.ros.org/independent/api/rosinstall/html/rosinstall_file_format.html>`__ and place it at the top level directory of your package. Its file name is your choice (typically this file is prefixed with a dot).

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
For example, let's say you want to test multiple distros (indigo, kinetic) and you have `.travis.rosinstall` and `.travis.rosinstall.kinetic` files in your repo. You can define the Travis config as:

::

    env:
      matrix:

        - ROS_DISTRO=indigo UPSTREAM_WORKSPACE=file
        - ROS_DISTRO=kinetic   UPSTREAM_WORKSPACE=file

With this config, for indigo default file name `.travis.rosinstall` will be seached and used if found. For kinetic, the file that consists of the default file name plus `.kinetic` suffix will be prioritized.

When `$ROSINSTALL_FILENAME.$ROS_DISTRO` file isn't found, `$ROSINSTALL_FILENAME` will be used for all jobs that define `UPSTREAM_WORKSPACE`.

Use .rosinstall from external location
++++++++++++++++++++++++++++++++++++++++++++++

You can utilize `.rosinstall` file stored anywhere as long as its location is URL specifyable. To do so, set its complete path URL directly to `UPSTREAM_WORKSPACE`.

Type of OS and distribution
--------------------------------------

Ubuntu and its distro are guessed by default from ROS_DISTRO
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

You can specify the OS and its distribution to run the CI job by setting `OS_NAME` and `OS_CODE_NAME`.
By default users don't need to set this and its value will be automatically guessed according to the value of `ROS_DISTRO`. e.g.::

  `ROS_DISTRO=indigo`  --> `OS_NAME=ubuntu` `OS_CODE_NAME=trusty`
  `ROS_DISTRO=kinetic` --> `OS_NAME=ubuntu` `OS_CODE_NAME=xenial`
  `ROS_DISTRO=lunar`   --> `OS_NAME=ubuntu` `OS_CODE_NAME=xenial`
  `ROS_DISTRO=melodic` --> `OS_NAME=ubuntu` `OS_CODE_NAME=bionic`

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

Possible combination of `OS_NAME` and `OS_CODE_NAME` depend on available Docker images. See `ros-industrial/docker/ci <https://github.com/ros-industrial/docker/tree/master/ci>`__.

Checking older ROS distros with industrial_ci
--------------------------------------------------------

For the older ROS distributions than `those that are supported <https://github.com/ros-industrial/industrial_ci#supported-ros-distributions>`__, you may still be able to use `industrial_ci`. Here's how to do so taking ROS `Hydro` as an example.

For `Travis CI`, you need at least the following changes in `.travis.yml`:

* Define `ROS_DISTRO` with  `hydro` (so have `ROS_DISTRO="hydro"`).

A successful example from `swri-robotics/mapviz <https://github.com/swri-robotics/mapviz/blob/49b0c5748950a956804e1976cfd7a224fa3f3f7d/.travis.yml>`__.

Run industrial_ci on local host
---------------------------------------

There are a few ways to run CI jobs locally.

Simplest way to run locally
++++++++++++++++++++++++++++++++

Since version 0.3.3, you can run `industrial_ci` on your local host. This can be useful e.g. when you want to integrate industrial_ci into your CI server.

NOTE that this way the CI config (e.g. `.travis.yml`, `.gitlab-ci.yml`) are not used. So whatever configurations you have in your CI configs need to be added manually.

To do so,

0. `Install Docker <https://docs.docker.com/engine/installation/linux/>`__
1. Build and install industrial_ci (which is `a catkin package <http://wiki.ros.org/ROS/Tutorials/CreatingPackage#ROS.2BAC8-Tutorials.2BAC8-catkin.2BAC8-CreatingPackage.What_makes_up_a_catkin_Package.3F>`__). Source setting.
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

Run locally using Travis config
++++++++++++++++++++++++++++++++

Since v0.6.0, you can run locally using `.travis.yml` you already defined for your repository, using [`industrial_ci/scripts/run_travis` script](https://github.com/ros-industrial/industrial_ci/blob/master/industrial_ci/scripts/run_travis). See the help of that script.

::

   rosrun industrial_ci run_travis --help

Recurring runs for debugging
++++++++++++++++++++++++++++
Please note that `run_ci` and `run_travis` will download all dependencies every time, just as CI services would do.
For recurring runs, e.g. in a debugging session, this might not be desired.

As an alternative `rerun_ci` could be used. It take the same argument as `run_ci` (note for `some limitations <#note-for-rerun-ci-limitations>`__), but will run the build incrementally and only download or compile after changes.

This results in much faster execution for recurring runs, but has some disadvantages as well:

* The user needs to clean-up manually, an instruction to do so is printed at the end of all runs.
* All parameters incl. the repository path have to be passed explicitly to allow for proper caching.
* The apt dependencies won't get updated in recurring runs.
* Incremental builds might not work properly for all cases. Especially, it does not help with prerelease tests.

Example:

::

  $ rosrun industrial_ci rerun_ci . ROS_DISTRO=melodic ROS_REPO=ros-shadow-fixed

This will run the tests and commit the result to a Docker image ``industrial-ci/rerun_ci/ros_canopen:$HASH``.
The hash is unique for each argument list, so ``rerun_ci . ROS_DISTRO=melodic`` and ``rerun_ci . ROS_DISTRO=kinetic`` do not mix  up.
However, it will keep consuming disk space with each new combination.

The cached images can be listed with
::

  $ rosrun industrial_ci rerun_ci --list

Note for rerun_ci limitations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

`rerun_ci` is managing `DOCKER_COMMIT` and `DOCKER_COMMIT_MSG` variables under the hood, so if the user set them they will not take effect, unlike `normal cases <#re-use-the-container-image>`__.

For maintainers of industrial_ci repository
================================================

Checks for industrial_ci repo itself
---------------------------------------

While this repository provides CI config that can be used by other repositories, it also checks this repo itself using the same CI config and the simplest package setting. That is why this repo contains the ROS package files and a test (`CMakeLists.txt`, `package.xml`, `.test`).
