^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.17 (2023-05-28)
-------------------

0.1.16 (2022-10-27)
-------------------
* remove roseus from CMakeLists.txt (`#773 <https://github.com/jsk-ros-pkg/jsk_control/issues/773>`_)
* [joint_states_appender.py]add queue_size (`#734 <https://github.com/jsk-ros-pkg/jsk_control/issues/734>`_)
* 0755 -> -h0o0755 : SyntaxError: leading zeros in decimal integer literals are not permitted; use an 0o prefix for octal integers, 2to3 -w -fprint . (`#763 <https://github.com/jsk-ros-pkg/jsk_control/issues/763>`_)
* Contributors: Kei Okada, Naoki Hiraoka

0.1.15 (2018-05-16)
-------------------

0.1.14 (2018-01-15)
-------------------

0.1.13 (2017-04-18)
-------------------

0.1.12 (2017-02-22)
-------------------
* CMakeLists.txt: remove pr2\_* dependency, add install
* Contributors: Kei Okada

0.1.11 (2017-02-09)
-------------------

0.1.10 (2016-12-15)
-------------------
* fix checker board spacing parameters to be 0.03
* [jsk_calibration] Modify checkerboard spacing values and modify README
* fix hrp2w_calibration
* update for hrp2w_calibration
* add readme comment
* change multisense -> xtion (or carmine)
* delete file before rename
* [jsk_calibration] Rename manual_calibration_viewer launch and fix colorize_points topic name
* add files for hrp2w
* [jsk_control/jsk_calibration] Fix typo.
* Contributors: Kohei Kimura, Yohei Kakiuchi

0.1.9 (2016-03-23)
------------------
* [jsk_calibration] Update camera config parameter for JAXON
  Modified:
  - jsk_calibration/jaxon_calibration/capture_data/hardware_config/cam_config.yaml
* [jsk_calibration] Add forgotten config file for rviz viewer
* [jsk_calibration] add utilitiy for jaxon_calibration
* [jsk_calibration][JAXON_RED] update poses for calibration
* [jsk_calibration] Add calibration viewer for manual calibration
* {jsk_calibration, jsk_footstep_planner}/README.md: fix section/subsection
* Contributors: Kei Okada, Ryohei Ueda, Yohei Kakiuchi, Iori Kumagai

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* [jsk_calibration] Fix typo in README
* Contributors: Ryohei Ueda

0.1.6 (2015-06-11)
------------------
* rename url
* [JAXON] add jaxon_calibration
* [JAXON] add motion generation method for jaxon camera calibration
* [jsk_calibration] Fix effort and velocity for joint_states which does not have effort and velocity
  in joint_states_appender.py
* [jsk_calibration] Update HRP2JSKNT calibration for multisense
* [jsk_calibration] Update README.md
* [jsk_calibration] append velocity and effort as well as name and position in joint_states_appender.py
* [jsk_calibration] Fix remapping of joint_states for HRP2JSKNTS
* [jsk_calibration] Fix documentation to show image
* [jsk_calibration] update documentation
* [jsk_calibration] Add launch files for hrp2jsknts calibration
* [jsk_calibration] Add hrp2jsknts to hrp2-calibration
* [jsk_calibration] Add joint_states_appender.py to handle multiple
  joint_states in calibration time.
* Updated to generate motion for hrp2w
* use least rpy angle for compatibility?
* add initial_poses.yaml for staro, and some update for staro calib
* [jsk_calibration] udpate parameters for staro
* fix board parmeter
* update poses for staro
* update generate-staro-files
* fix camera joint name
* update camera name
* [jsk_calibration] Fix robust-motion bug and not use it as default
* [jsk_calibration] Do not check collision if IK failed and fix unit of initial_poses.yaml
* [jsk_calibration] Fix :write-initial-pose-yaml by Yohei's patch.
* [jsk_calibration] Write initial_poses.yaml to speed up optimization
* update
* [jsk_calibration] Fix typoe of the files
* add estimate params
* fix chain length
* add staro_calibration

0.1.5 (2015-01-08)
------------------
* Add utility script to test a lot of calibration parameters
* Reduce sampling poses to 30
* Sampling pose with configuration space and update sampling space of
  leg poses
* Add legs to calibrate hrp2jsknt
* Self Collisoin with small changes on joint angles to get stable position
* Add calibration document
* Select different poses with probability computation
* Select good calibration sample from a lot of mother samples
* Use dual arm to calibrate HRP2JSKNT
* Filtering not good poses for calibration
* Do not use gripper joint on HRP2JSKNT
* Add rviz config to see pose_guesses marker
* rename make-calibration-pose.l -> calibration.l and separate hrp2
  specific setting to hrp2-calibration.l
* Remove unused file
* Update calibration for hrp2jsknt
* Add motion files for hrp2jsknt and implement several methods to generate yamls and launches
* Generate motion with more strict collision check and add test method on real robot
* Update parameters for hrp2jsknt
* Add special reset-pose for leg calibration
* Remove unused files
* Add hrp2jsknt_calibration directory just copied from hironx_calibration
* Add board to collision check and refine motion with interpolated angle vectors
* Add method to check collision of interpolated motions
* Add more depends
* add legs motion
* Implement hrp2 motion generation
* Depends on pr2_msgs and pr2_controllers_msgs
* Add jsk_calibration package for hand-eye calibration
* Contributors: Ryohei Ueda

0.1.4 (2014-10-21)
------------------

0.1.3 (2014-10-10)
------------------

0.1.2 (2014-09-08)
------------------

0.1.1 (2014-09-04)
------------------
