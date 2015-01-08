^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
