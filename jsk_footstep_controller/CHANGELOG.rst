^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_footstep_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2015-06-11)
------------------
* [jsk_footstep_controller] Fix typo
* [jsk_footstep_controller] Finally fixed
* [jsk_footstep_controller] FInally fix odom on ground
* [jsk_footstep_controller] Call adjust-foot-steps before set-foot-steps
* [jsk_footstep_controller] Fix parameters for jaxon red
* [jsk_footstep_planner] Support JAXON_RED
* [jsk_footstep_controller] Fix odom_root yaw orientation
* [jsk_footstep_controller] Fix footcoords initialization
* [jsk_footstep_planner, jsk_footstep_controller] Update for las vegas terrain with jaxon
* [jsk_footstep_controller] Update gait generator parameter for jaxon
* [jsk_footstep_controller] Update parameter for jaxon
* [jsk_footstep_controller] Fix resolvance of odom orientation
* [jsk_footstep_controller] Automatically lookaround ground after finishing walking
* [jsk_footstep_controller] Add :semi-interruptible mode
* [jsk_footstep_controller] Decide file to load from ROBOT environmental variable
* [jsk_footstep_controller] Add semi/full/non interruptible mode
* [jsk_footstep_planner, jsk_footstep_controller] Update for jaxon terrain
* [jsk_footstep_controller] Change gait generator parameters according to
  plane condition including pitch angle and taking into account
  if transition is upward or downward
* [jsk_footstep_planner, jsk_footstep_controller] Compute x and y from
  previous coordinates to detect rolled plane
* [jsk_footstep_planner, jsk_footstep_controller] Support rolling terrain, I hope
* [jsk_footstep_planner, jsk_footstep_controller] Fix typo and update parameter for terrain
* [jsk_footstep_controller/lookaround-ground] Do not stretch knee if HRP2 has toe joints
* [jsk_footstep_controller/footcoords] More readable error output
* [jsk_footstep_planner, jsk_footstep_controller] Support jaxon parameter files
* [jsk_footstep_controller] Remove unused parameter
* [jsk_footstep_planner, jsk_footstep_controller] Support jaxon footstep planning, Do not merge yet
* [jsk_footstep_controller] Support jaxon motion
* [jsk_footste_controller] Fix odom_on_ground consistency but ignore correctness during single stance phase
* need down-case for URATA robot
* [jsk_footstep_controller] Support hrpsys robots in lookaround-ground.l like JAXON
* [jsk_footstep_controller] Do not update odom_on_ground in single stance phase
* [jsk_footstep_controller] Remove sleep in lookaround-ground.l
* [jsk_footstep_controller] Update according to the latest :get-foot-step-param of rtm-robot-interface
* [jsk_footstep_controller] Update ground and odom_on_ground periodically rather than synchronizing with force sensors.
  Just update contact states from force sensors
* [jsk_footstep_controller] Use the latest best gait generator parameters
* [jsk_footstep_controller] Re-work interruptible (step-by-step) walking mode
* [jsk_footstep_controller] Add sleep to wait for st convergence in lookaround-ground.l
* [jsk_footstep_controller] Update AutoBalancer parameter
* [jsk_footstep_controller] Remove read-line for debug in lookaround-ground.l
* [jsk_footstep_controller] Update pose to look ground: Streching knee and
  specify wait pitch directly
* [jsk_footstep_controller] Support HRP2JSK, HRP2JSKNT and HRP2JSKNTS
* [jsk_footstep_planner] A lot of improvements on planner
  * Fix about orientation of projected footstep
  * Lazy evaluation to check if footprint is able to be on grid
  * Use mid-coords of footprints to evaluate heuristic
* [jsk_footstep_planner] Support SimpleOccupancyGrid in footstep planner
* [jsk_footstep_controller] Wait for tf transformation to resolve force sensor transformation.
* [jsk_footstep_controller] Check tf2::LookupException
* [jsk_footstep_controller] transform force vector to specified frame_id in footcoords
* [jsk_footstep_coords] Do not update transformation between ground and odom on ground during dual leg stance phase

0.1.5 (2015-01-08)
------------------
* Update drcmodel for current planner
* Fix poping-up cancel window by broadcasting canceled information
* Change threshold according to the footsteps respectively
* Wait until contact state is stable during interrubtible-walking
* Check contact state is stable or not in footcoords.cpp
* Apply low-pass filter to force sensor values
* Add script to compute stats about contact_states
* Add text publisher about single/double stance phase
* Merge remote-tracking branch 'origin/master' into add-breakpoint-text
  Conflicts:
  jsk_footstep_controller/euslisp/footstep-controller.l
  jsk_footstep_controller/launch/hrp2jsknt_real_full.launch
* Add text publishing when checking breakpoint
* Update forth threshold to 25N to regard the leg is on floor
* Add z-error to contact_state of footcoords
* check tf2::ExtrapolationException in footcoords
* Change walking orbit and the height of the root link according to the plans
* Use snapit to snap the goal of footstep to the planes
* Change the color of footsteps if there is no planning result
* Update footstep parameter for climing up stairs:
  larger footstep and smaller footprint
* Publish usage of footstep planner joy
* Publish conctact state and angular error between two legs as topic
* Publish support leg information to diagnostic
* During single support phace, ground should on the end effector coordinates
* Add documentation about footcoords
* Publish /odom_on_ground and /ground tf frame from footcoords
* Fix indent of footcoords
* Display footstep parameter on rviz
* Move down 50 mm during walking and use more larger step for walking
* Fix calculation of roll difference
* Separate roll and pitch angles to calculate angular difference between
  footstep to be refined
* Fix refinment of footstep by using relative transformation to the
  previous footstep
* Refine result of footstep planning by filtering goal of actionlib interface
  of footstep planner.
* Reset to reset-manip-pose after look around the ground
* Contributors: Ryohei Ueda

0.1.4 (2014-10-21)
------------------
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
