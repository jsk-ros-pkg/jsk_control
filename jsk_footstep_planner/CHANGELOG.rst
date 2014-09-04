^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_footstep_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-09-04)
------------------
* use lock/unlock service of environment server to lock/unlock the environment during planning
* compile euslisp file before running footstep planner
* publish footstep for visualization from planner
* update usage of env server according to the latest changeset of
  jsk_recognition
* use env server of jsk_pcl_ros
* ignore emtpy polygon message
* prepend initial steps to the result of the footstep planning
* call x::window-main-onw only if *debug* is t in jsk_footstep_planner/footstep-planner-node.l
* support 6dof planning
* adding model for footstep planning
* finalize footstep by goal steps
* supporting slope in footstep planning
* update for slope planning
* begins to support slope
* automatically choose the goal footstep
* store goal footstep to the problem class
* supress debug message of footstep planner
* update python scripts for catkin
* load msgs directory
* fix dependency
* keep permission of euslisp codes
* catkinize jsk_footstep_planner
* fix to keep orientation after projection to the planes
* supporting z-direction movement in planning
* supporting timeout of planning
* adding jsk_footstep_planner, euslisp implementation
* Contributors: Ryohei Ueda, Masaki Murooka
