^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_ci
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2018-01-13)
------------------
* [capability] You can run industrial_ci locally on your computer using configuration in your .travis.yml `#230 <https://github.com/ros-industrial/industrial_ci/issues/230>`_
* [capability] Inject QEMU `#237 <https://github.com/ros-industrial/industrial_ci/issues/237>`_ from ipa-mdl/feature/inject-qemu
* [improve] Easier to use your custom Docker image `#259 <https://github.com/ros-industrial/industrial_ci/issues/259>`_
* Contributors: Isaac I.Y. Saito, Mathias Lüdtke

0.5.1 (2017-12-29)
------------------
* [fix] Workaround for the regression in "docker cp" `moby/moby#34096 <https://github.com/moby/moby/issues/34096>`_ (`#248 <https://github.com/ros-industrial/industrial_ci/issues/248>`_).
  * Gitlab CI users need to take an action. See `this post <https://github.com/ros-industrial/industrial_ci/pull/248#pullrequestreview-85975989>`_
* [improve] Verbose catkin test results `#251 <https://github.com/ros-industrial/industrial_ci/issues/251>`_
* Contributors: Mathias Lüdtke

0.5.0 (2017-12-12)
------------------
* [fix] ROS Prerelease Test by updating to the latest Docker CE `#238 <https://github.com/ros-industrial/industrial_ci/pull/238>`_
* [capability] ABI check based on abi-compliance-checker `#199 <https://github.com/ros-industrial/industrial_ci/issues/199>`_
* [capability] catkin_lint feature. `#219 <https://github.com/ros-industrial/industrial_ci/issues/219>`_
* [capability] implemented ROSDEP_SKIP_KEYS feature `#184 <https://github.com/ros-industrial/industrial_ci/pull/184>`_
* [enhance] Add support for ARM platform `#236 <https://github.com/ros-industrial/industrial_ci/issues/236>`_
* Many more bug fixes.
* Contributors: Mathias Lüdtke, Miguel Prada, Isaac I.Y. Saito

0.4.0 (2017-06-29)
------------------
* [capability] ROS buildfarm-powered pre-release tests can now run for even the repositories that are not listed in rosdistro https://github.com/ros-industrial/industrial_ci/pull/145
* [capability] support for ROS lunar
* [capability] Enable support for gitlab `#120 <https://github.com/ros-industrial/industrial_ci/issues/120>`_
* [capability] OS selection option https://github.com/ros-industrial/industrial_ci/pull/174
* [capability] Option for reusing build artifact by ccache https://github.com/ros-industrial/industrial_ci/pull/182
* [capability] Allow subversion in .rosintall file https://github.com/ros-industrial/industrial_ci/pull/179
* [enhance] Speed up jobs by building into install space directly https://github.com/ros-industrial/industrial_ci/pull/150    
* [enhance] Allow passing 'catkin config' parameters (addresses `#176 <https://github.com/ros-industrial/industrial_ci/issues/176>`_). https://github.com/ros-industrial/industrial_ci/pull/177
* [enhance] apt/deb handling in docker image https://github.com/ros-industrial/industrial_ci/issues/164, https://github.com/ros-industrial/industrial_ci/pull/158
* Contributors: Benjamin Maidel, Isaac I.Y. Saito, Iñigo Martínez, Jon Azpiazu, Mathias Lüdtke

0.3.3 (2017-02-09)
------------------
* [capability] Added a script to run industrial_ci on a local host in a Docker container. `#116 <https://github.com/ros-industrial/industrial_ci/pull/116>`_.
* Contributors: Mathias Lüdtke

0.3.2 (2017-01-20)
------------------
* [capability] New variables: CATKIN_WORKSPACE, DEBUG_BASH, EXPECT_EXIT_CODE, AFTER_SCRIPT (see `document <https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst>`_ for detail)
* [capability] Choose verbose output `#94 <https://github.com/ros-industrial/industrial_ci/issues/94>`_
* [capability] enable/disable verbose output (`#94 <https://github.com/ros-industrial/industrial_ci/issues/94>`_)
* [capability] Support private github repositories on kinetic (`#92 <https://github.com/ros-industrial/industrial_ci/issues/92>`_)
* [fix, capability] Script terminates when rosdep install fails. Remove manifest.xml handling `#95 <https://github.com/ros-industrial/industrial_ci/issues/95>`_
* [fix][ci_main.sh] Fix to not terminate falsely. (`#84 <https://github.com/ros-industrial/industrial_ci/issues/84>`_)
* [fix] ROS prerelease test using code in the pull request. (`#85 <https://github.com/ros-industrial/industrial_ci/pull/85>`_)
* [fix] ROS prerelease test on Kinetic. (`#83 <https://github.com/ros-industrial/industrial_ci/pull/83>`_)
* [fix] Kinetic testing fix (environment variables passed to docker container) (`#82 <https://github.com/ros-industrial/industrial_ci/issues/82>`_)
* [fix] Many build/install issues https://github.com/ros-industrial/industrial_ci/pull/110, https://github.com/ros-industrial/industrial_ci/pull/109 (`#92 <https://github.com/ros-industrial/industrial_ci/issues/92>`_)
* [doc] Split readme into quick start and detail. More beginner doc. https://github.com/ros-industrial/industrial_ci/pull/113
* [maintenance] Change license to Apache 2.0 (addresses `#17 <https://github.com/ros-industrial/industrial_ci/issues/17>`_).
* [maintenance] Major code refactoring. Generalizing function and variable names (removing mention to specific CI system) https://github.com/ros-industrial/industrial_ci/pull/108
* [maintenance] Add maintainer.
* Contributors: Benjamin Maidel, Isaac I.Y. Saito, Mathias Lüdtke

0.3.1 (2016-10-24)
------------------
* [fix] usermod error on docker-based ROS Prerelease (see https://github.com/ros-industrial/ros_canopen/pull/193#issuecomment-254575036). (`#81 <https://github.com/ros-industrial/industrial_ci/issues/81>`_)
* Contributors: Mathias Lüdtke, Isaac I.Y. Saito

0.3.0 (2016-09-07)
------------------
* [fix] Catch apt error for ADDITIONAL_DEB (Fix `#78 <https://github.com/ros-industrial/industrial_ci/issues/78>`_). (`#79 <https://github.com/ros-industrial/industrial_ci/issues/79>`_)
* [feat] ROS Hydro compatible (Only use catkin_test_results --verbose if it exists, `#77 <https://github.com/ros-industrial/industrial_ci/issues/77>`_)
* [feat] Allow failure for now the jade source build (see https://github.com/ros-industrial/industrial_core/pull/144#issuecomment-223186764).
* [improve] Use install space by default (addresses `#54 <https://github.com/ros-industrial/industrial_ci/issues/54>`_).
* [maintenance] Refactoring `#67 <https://github.com/ros-industrial/industrial_ci/pull/67>`_
* Contributors: Dave Coleman, Robert Haschke Edward Venator, Isaac I.Y. Saito

0.2.2 (2016-05-13)
------------------
* [fix] Remove wrong duplicate prerelease test code block. `#40 <https://github.com/ros-industrial/industrial_ci/issues/40>`_
* [sys] Adjust to ROS Indigo's up-to-date ros.key acquision. `#42 <https://github.com/ros-industrial/industrial_ci/issues/42>`_
* Contributors: Isaac I.Y. Saito, Gijs van der Hoorn, Mathias Lüdtke

0.2.1 (2016-05-06)
------------------
* [feat] Add docker-based ROS prerelease test. `#35 <https://github.com/ros-industrial/industrial_ci/issues/35>`_
* [fix] Correct environment variable exportation to subprocesses.
* [fix] Better script termination with 'set -e'.
* [fix] broken link in README `#37 <https://github.com/ros-industrial/industrial_ci/issues/37>`_
* [fix] apt-get quiet option `#33 <https://github.com/ros-industrial/industrial_ci/issues/33>`_
* [sys] Extract util functions
* [sys] Remove meaningless Travis jobs
* [doc] Some clarifications.
* [improve] More fold Travis result (wstool version and localname info) `#38 <https://github.com/ros-industrial/industrial_ci/issues/38>`_
* Contributors: Mathias Lüdtke, Dave Coleman, Victor Lamoine, Isaac I.Y. Saito

0.2.0 (2016-04-19)
------------------
* Adjust to catkin_tools 0.4.0 `#31 <https://github.com/ros-industrial/industrial_ci/issues/31>`_
* Contributors: Isaac I.Y. Saito

0.1.3 (2016-04-14)
------------------
* [fix] Temporarilly disable `rospack plugin` line (fixes `#26 <https://github.com/ros-industrial/industrial_ci/issues/26>`_). `#28 <https://github.com/ros-industrial/industrial_ci/issues/28>`_
* [fix] missing an arg for specifying the number parallel job.
* Fix undeclared args for the number parallel job `#22 <https://github.com/ros-industrial/industrial_ci/issues/22>`_
* [doc] Clarify parallel job args.
* Contributors: Isaac I.Y. Saito

0.1.2 (2016-02-08)
------------------
* [fix] Move a patch that becomes available via DEB to older ROS distro only section (`#20 <https://github.com/ros-industrial/industrial_ci/issues/20>`_)
* [feat] Add option to not test (`#16 <https://github.com/ros-industrial/industrial_ci/issues/16>`_)
* Contributors: Isaac I.Y. Saito, Gijs van der Hoorn

0.1.1 (2016-01-05)
------------------
* [feat] Better variable name for downstream pkgs
* [doc] Many improvements including replacing "git submodule" with "git clone"
* [enhance] Output enhancement and cleanup
* [enhance] Turn off status line (`#4 <https://github.com/ros-industrial/industrial_ci/issues/4>`_)
* [sys] Remove a tentative workaround for a test location issue (https://github.com/ros/ros_comm/pull/668)
* Contributors: Isaac I.Y. Saito, Mathias Lüdtke

0.1.0 (2015-12-08)
------------------
* Init commit of travis config and scripts
* Add license and copyright header
* Contributors: Shaun Edwards, Isaac I.Y. Saito
