# Migration guide

This guide covers the migration from the [legacy](https://github.com/ros-industrial/industrial_ci/tree/legacy) version.

## What's new?

The new version comes with a number of improvements:

* Added support for catkin_make, catkin_make_isolated and colcon
* Use [vcstool](https://github.com/dirk-thomas/vcstool)(`vcs`) instead of `wstool` to support \*.rosinstall and \*.repos files
* Added support for ROS2, and vcstool (rosinstall or repos files)
* Separate upstream and downstream [workspaces](index.rst#workspace-management), even for prerelease tests
* Fine-grained [hook system](index.rst#customize-within-the-ci-process) for customization
* Writable target folder

## What's the catch?

Some features are not supported anymore:

* Support of ROS hydro
* Devel space builds
* Testing installed \*.test files
* Injecting QEMU (see [`INJECT_QEMU`](#inject_qemu))
* Job control settings have been removed for now

If you depend on these, you can still use the [legacy](https://github.com/ros-industrial/industrial_ci/tree/legacy) version.

## How can I migrate?

First of all, you have to switch to the master branch in your CI config:
```
git clone --quiet --depth 1 https://github.com/ros-industrial/industrial_ci.git .industrial_ci -b master
```

There is a big chance that the new version works out of the box.
However, the workspace layout and the locations are changed.

Special care must be taken, if you use any of these variables:

* [`ABICHECK_URL`](#abicheck_url)
* [`ADDITIONAL_DEBS`](#additional_debs)
* [`AFTER_SCRIPT`](#after_script)
* [`APTKEY_STORE_HTTPS`](#aptkey_store_https)
* [`APTKEY_STORE_SKS`](#aptkey_store_sks)
* [`AFTER_SCRIPT`](#after_script)
* [`BEFORE_SCRIPT`](#before_script)
* [`BUILD_PKGS_WHITELIST`](#build_pkgs_whitelist)
* [`BUILDER`](#builder)
* [`CATKIN_CONFIG`](#catkin_config)
* [`CATKIN_PARALLEL_JOBS`](#catkin_parallel_jobs)
* [`CATKIN_PARALLEL_TEST_JOBS`](#catkin_parallel_test_jobs)
* [`CMAKE_ARGS`](#cmake_args)
* [`DOCKER_BASE_IMAGE`](#docker_base_image)
* [`DOCKER_FILE`](#docker_file)
* [`DOCKER_IMAGE`](#docker_image)
* [`HASHKEY_SKS`](#hashkey_sks)
* [`NOT_TEST_INSTALL`](#not_test_install)
* [`INJECT_QEMU`](#inject_qemu)
* [`PKGS_DOWNSTREAM`](#pkgs_downstream)
* [`ROSINSTALL_FILENAME`](#rosinstall_filename)
* [`ROS_PARALLEL_JOBS`](#ros_parallel_jobs)
* [`ROS_PARALLEL_TEST_JOBS`](#ros_parallel_test_jobs)
* [`ROS_REPOSITORY_PATH`](#ros_repository_path)
* [`TARGET_PKGS`](#target_pkgs)
* [`UBUNTU_OS_CODE_NAME`](#ubuntu_os_code_name)
* [`UPSTREAM_WORKSPACE`](#upstream_workspace)
* [`USE_DEB`](#use_deb)
* [`VERBOSE_OUTPUT`](#verbose_output)
* [`VERBOSE_TESTS`](#verbose_tests)
* [`USE_MOCKUP`](#use_mockup)

## Changes summary

### Docker images

Instead of creating a new Docker image on every build, `industrial_ci` will try to use the [official ROS images](https://hub.docker.com/_/ros/), if possible (ROS kinetic and newer, ROS_REPO=ros/ros2).
You can set `DEFAULT_DOCKER_IMAGE=''` to opt-out.

### Workspace layout

The workspace layout has changed (new [workspace management](index.rst#workspace-management)).
The target workspace is now located in `~/target_ws`.
The upstream workspace packages are now located in `~/upstream_ws`.

### Job control

`industrial_ci` does not set defaults for the number of jobs anymore.
In addition the job-control options got removed for the time being.

### Hook system

The customization support via `BEFORE_SCRIPT` and `AFTER_SCRIPT` has been refactored into a flexible [hook system](index.rst#customize-within-the-ci-process), which offers script support for every step in the workflow.

The new system can read exposed variables and use the `industrial_ci` functions.
The [workspace layout has changed](#workspace-layout), so your scripts might need to get adapted accordingly.
To simplify the migration the script hooks can read the workspace location from the `base_ws` (ABI check only), `downstream_ws`,`target_ws` and `upstream_ws` variables.
In addition steps that deal with workspaces expose the `current_ws` variable.
Please note that the variables can be used in the script option itself, but not in invoked scripts.
You have to pass them into your script, e.g.
```
AFTER_SETUP_UPSTREAM_WORKSPACE='./my_script.sh "$current_ws"'
```
(*For Gitlab CI you have to use `$$variable` to prevent the premature substitution*)

Furthermore, the hook will get run without a ROS environment (`setup.bash`).
If you need this environment, you can use the `rosenv` helper.
Optionally, it takes a command to be executed.

Examples:

* `AFTER_SETUP_UPSTREAM_WORKSPACE='rosenv && echo "$ROS_DISTRO'"`
* `AFTER_SETUP_UPSTREAM_WORKSPACE='rosenv ./my_script.sh'`

## Affected variables

### ABICHECK_URL

Everything should work as before, but git-protocol will get used by default.
In addition, the extended [definition syntax](index.rst#workspace-definition) is supported.

### ADDITIONAL_DEBS

This variable is still supported and is now implemented for all tests.

### AFTER_SCRIPT

This variable is still supported, but the [script environment has changed](#hook-system).

### APTKEY_STORE_HTTPS

It is not a fallback for [`APTKEY_STORE_SKS`](#aptkey_store_sks) anymore, but will disable it and download the key from the provided URL directly.

### APTKEY_STORE_SKS

Will only be used if [`APTKEY_STORE_HTTPS`](#aptkey_store_https) is not set, without a fall-back.

### BEFORE_SCRIPT

Due to the new [workspace management](index.rst#workspace-management), the `BEFORE_SCRIPT` variable became ambiguous and was therefore removed.
However, the new [hook system](index.rst#customize-within-the-ci-process) offers a lot of alternatives.

If you did not use [`UPSTREAM_WORKSPACE`](#upstream_workspace)(other than `debian`) or [`USE_DEB=false`](#use_deb), you can just substitute `BEFORE_SCRIPT` with `AFTER_SETUP_TARGET_WORKSPACE`. This will execute your script in the same sequence as before.
If you did use the upstream settings, you might need to substitute with `AFTER_SETUP_UPSTREAM_WORKSPACE` (or both).

In general you can as well use any other hook that is suitable for your script.
Please note that the [script environment has changed](#hook-system).

### BUILD_PKGS_WHITELIST

This variable is not supported anymore.
Instead, you could remove the unrelated folders (see [workspace management](index.rst#workspace-management)).

### BUILDER

This variable was not used before, but it was listed as an option.
If you specified it anyway, please set it to `catkin_tools` or remove it to allow for automatic selection.

### CATKIN_CONFIG

This variable is not supported anymore.
As an alternative you could set:
* `CC` and `CFLAGS` for C compiler settings
* `CXX` and `CPPFLAGS`/`CXXLAGS` for C++ compiler settings
* `CMAKE_ARGS` for CMake settings

If you need catkin_tools-specific settings, especially `--no-install`, you should use the [legacy](https://github.com/ros-industrial/industrial_ci/tree/legacy) version.
Or even better: Add support for the install space.

### CATKIN_PARALLEL_JOBS

This is not supported anymore (see [job control](#job-control)) as well.
Just try your build without this setting.
If it does not work, you should review the package dependencies in your repository.

### CATKIN_PARALLEL_TEST_JOBS

This is not supported anymore (see [job control](#job-control)) as well.
Just try your build without this setting.
Please [open an issue](https://github.com/ros-industrial/industrial_ci/issues/new) if you really depend on this.

### CMAKE_ARGS

This variable was introduced recently and is still supported, but its content will be passed to all workspaces.
`DOWNSTREAM_CMAKE_ARGS`, `TARGET_CMAKE_ARGS` and `UPSTREAM_CMAKE_ARGS` can be used for more fine-grained control.

### DOCKER_BASE_IMAGE

This variable is still supported, but the [workspace layout has changed](#workspace-layout).

### DOCKER_FILE

This variable is still supported, but the [workspace layout has changed](#workspace-layout).

### DOCKER_IMAGE

This variable is still supported, but the [workspace layout has changed](#workspace-layout).

### HASHKEY_SKS

Will only be used if [`APTKEY_STORE_HTTPS`](#aptkey_store_https) is not set.

### INJECT_QEMU

This option was removed. It is not needed for newer versions of `qemu-user-static` (host OS: Ubuntu cosmic or newer, Debian buster or newer).
For older versions, please try
```
docker run --rm --privileged multiarch/qemu-user-static --reset --credential yes --persistent yes
```

### NOT_TEST_INSTALL

This feature was removed, please use the [legacy](https://github.com/ros-industrial/industrial_ci/tree/legacy) version.

### PKGS_DOWNSTREAM

This variable is not supported anymore.
Instead, you could specify `DOWNSTREAM_WORKSPACE` (see [workspace management](index.rst#workspace-management)).

### ROSINSTALL_FILENAME

This variable is deprecated, and should get migrated.

If you do not set `UPSTREAM_WORKSPACE=file` as well, `ROSINSTALL_FILENAME` can just be removed.
Otherwise, set `UPSTREAM_WORKSPACE` to the value of `$ROSINSTALL_FILENAME` or `$ROSINSTALL_FILENAME.$ROS_DISTRO` explicitly (no auto-branching anymore).
If your rosinstall file contains your target repository as well, you might want to [filter](index.rst#workspace-management) it out:
`UPSTREAM_WORKSPACE=my_file_name -path_to_target_repo`.

### ROS_PARALLEL_JOBS

This variable does not get processed by `industrial_ci` anymore, instead it just gets passed to the build tool.
Just try your build without this setting.
If it does not work, you should review the package dependencies in your repository.

### ROS_PARALLEL_TEST_JOBS

This is not supported anymore (see [job control](#job-control)) as well.
Just try your build without this setting.
Please [open an issue](https://github.com/ros-industrial/industrial_ci/issues/new) if you really depend on this.

### ROS_REPOSITORY_PATH

This variable is supported as-is.
If it is set to `http://packages.ros.org/ros-shadow-fixed/ubuntu` or `http://packages.ros.org/ros-testing/ubuntu`,  it can be removed completely or replaced by `ROS_REPO=ros-testing` (default).
If it is set to `http://packages.ros.org/ros/ubuntu`, it could get shortened to `ROS_REPO=ros` to make `industrial_ci` use the [official ROS Docker](https://hub.docker.com/_/ros) images, if possible.

### TARGET_PKGS

This variable is not supported anymore.
Instead, you could remove the unrelated folders (see [workspace management](index.rst#workspace-management)).

### UBUNTU_OS_CODE_NAME

Still works, but is deprecated. Just substitute it with `OS_CODE_NAME`

### UPSTREAM_WORKSPACE

`UPSTREAM_WORKSPACE=debian` is superfluous and can simply get removed.
In case of `UPSTREAM_WORKSPACE=file` it can get set to the value of `$ROSINSTALL_FILENAME` (default: .travis.rosinstall) or `$ROSINSTALL_FILENAME.$ROS_DISTRO` explicitly (no auto-branching anymore).

If your rosinstall file contains your target repository as well, you might want to [filter](index.rst#workspace-management) it out:
`UPSTREAM_WORKSPACE=my_file_name_or_URL -path_to_target_repo`.

### USE_DEB

`USE_DEB=true` is superfluous and can simply get removed.
`USE_DEB=false` is the same as [`UPSTREAM_WORKSPACE=file`] and should [get migrated](#upstream_workspace).

### USE_MOCKUP

Is not used anymore, just substitute it with `TARGET_WORKSPACE`

### VERBOSE_OUTPUT

Is only implemented for `BUILDER=catkin_tools`

### VERBOSE_TESTS

Is only implemented for `BUILDER=catkin_tools`
