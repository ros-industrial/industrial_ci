^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_ci
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2016-05-13)
------------------
* [fix] Remove wrong duplicate prerelease test code block. `#40 <https://github.com/ros-industrial/industrial_ci/issues/40>`_
* [sys] Adjust to ROS Indigo's up-to-date ros.key acquision. `#42 <https://github.com/ros-industrial/industrial_ci/issues/42>`_
* Contributors: Isaac I.Y. Saito, Gijs van der Hoorn, Mathias Luedtke

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
* Contributors: Mathias Luedtke, Dave Coleman, Victor Lamoine, Isaac I.Y. Saito

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
* Contributors: Isaac I.Y. Saito, Mathias Luedtke

0.1.0 (2015-12-08)
------------------
* Init commit of travis config and scripts
* Add license and copyright header
* Contributors: Shaun Edwards, Isaac I.Y. Saito
