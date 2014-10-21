^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_footstep_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Refine footsteps to snapped to plane
* Add simple motion to look around the floor near from legs and
  update minor stuff for the latest EnvironmentPlaneModeling

0.1.3 (2014-10-10)
------------------
* Add footcoords to jsk_footstep_controller to compute tf like "/odom on ground"
  by monitoring foot force sensors
* do not run foot_contact_monitor in hrp2jsknt_real.launch. that script will be launched in default startup launch file

0.1.2 (2014-09-08)
------------------

0.1.1 (2014-09-04)
------------------
* fix the menu when walking is canceled and update the pose from joy stick according
  to the snapped pose availble by marker
* pop menu when cancel the footstep and support resuming from joystick
* cancel walking via joystick
* update diagnostics information about footstep planning and joy stick stuff
* add diagnostics.yaml for footstep environment
* add diagnostics_aggregator and use ps3joy in hrp2jsknt_real.launch
* add foot contact monitor and initialize the pose of the footstep_marker in hrp2jsknt_real.launch
* publish diagnostic status according to the contact state of the feet
* add a script to publish /ground frame according to the contact state of the feet
* interruptible footstep controller
* Merge pull request `#52 <https://github.com/jsk-ros-pkg/jsk_control/issues/52>`_ from garaemon/update-env-server
  update usage of env server in footstep planner according to the latest changeset of jsk_recognition
* update usage of env server according to the latest changeset of
  jsk_recognition
* support multiple instances per one plugin class
* add interface to get log of footstep
* use env server of jsk_pcl_ros
* wait controller until it's activated with infinite timeout
* add a launch file to start footstep stuff for real robot
* fix transformations of coordinates of jsk_footstep_controller
* run sample only one time
* add more debug messages
* fix transformations
* transform footstep relative to hrpsys coordinate system
* use the first step to adjust coordination system, not use offset parameter
  in footstep-controller
* refactoring footstep-controller.l
* make the codes within 80 columns: footstep-controller.l
* fix syntax of footstep-controller.l
* foostep_controller: apply offset specified by rosparam
* read end-coords-offset in footstep-controller
* use config file in sample launch file and add that config file
* update footstep successors parameters
* add autonomous sample launch file
* update several successors parameters
* prepend initial footstep and start st first
* remove dumb lines to shorten code: footstep-controller.l
* fix the argument of execute-cb and fix several trivial syntax errors
* add footstep_controller to sample launch file
* specify offset and frame_id of the legs to JoyFootstepPlanner
* update the foot offset parameter
* add sample launch file for hrp2jsknt
* remove comment from package.xml
* add manifest.xml to jsk_footstep_controller
* install launch directory of jsk_footstep_controller
* add script to move pose only
* instantiating ros bridge client
* controller to execute footstep on hrpsys
* Contributors: Ryohei Ueda
