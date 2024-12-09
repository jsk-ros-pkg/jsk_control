^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_footstep_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.18 (2024-12-09)
-------------------

0.1.17 (2023-05-28)
-------------------

0.1.16 (2022-10-27)
-------------------
* [jsk_footstep_controller] add head-controller.l, add base-controller.l (`#758 <https://github.com/jsk-ros-pkg/jsk_control/issues/758>`_)
* [jsk_footstep_controler/robot-boundingbox.l] remove dependency on jsk_demos/drc_task_common (`#764 <https://github.com/jsk-ros-pkg/jsk_control/issues/764>`_)
* [jsk_control]add queue_size (`#735 <https://github.com/jsk-ros-pkg/jsk_control/issues/735>`_)
* [jsk_footstep_controller] add sole conf for JAXON_BLUE (`#690 <https://github.com/jsk-ros-pkg/jsk_control/issues/690>`_)
* 2to3 -w -fprint . (`#763 <https://github.com/jsk-ros-pkg/jsk_control/issues/763>`_)
* Add melodic support [jsk_footstep_controller] fix a bug in footcoords.cpp due to a bug of boost::assign::list_of (`#740 <https://github.com/jsk-ros-pkg/jsk_control/issues/740>`_)
* [jsk_footstep_controller] install config (`#725 <https://github.com/jsk-ros-pkg/jsk_control/issues/725>`_)
* jsk_footstep_controller: check SoundRequest  has volume attribute (`#726 <https://github.com/jsk-ros-pkg/jsk_control/issues/726>`_, `#724 <https://github.com/jsk-ros-pkg/jsk_control/issues/724>`_)
* add missing install directories (`#699 <https://github.com/jsk-ros-pkg/jsk_control/issues/699>`_)

* Contributors: Kei Okada, Koki Shinjo, Masaki Murooka, Naoki Hiraoka, Yasuhiro Ishiguro, Kunio Kojima

0.1.15 (2018-05-16)
-------------------

0.1.14 (2018-01-15)
-------------------

0.1.13 (2017-04-18)
-------------------
* CMakeLists.txt : fix typo on install DESTINATION
* [jsk_footstep_controller] prevent too large footstep refine
* [jsk_footstep_controller] add utilitiy functions and update for footstep_planner
* [jsk_footstep_controller] add keyword for sample footstep client
* [jsk_footstep_contorller] Do not publish odom tf when publish_odom_tf param is false
* [jsk_footstep_contorller] Update footstep params for JAXON
* Contributors: Kei Okada, Yohei Kakiuchi, Iori Kumagai

0.1.12 (2017-02-22)
-------------------

0.1.11 (2017-02-09)
-------------------

0.1.10 (2016-12-15)
-------------------
* [jsk_footstep_controlelr/euslisp/util.l] Add function to gen pointcloud from face list
* [jsk_footstep_controller/euslisp/util.l] Set lfoot rfoot offset based on robot model.
* [jsk_footstep_controller/euslisp/util.l] Add function to get pointcloud included in bodies
* [jsk_footstep_controller/euslisp/util.l,jsk_footstep_planner/euslisp/footstep-planner-client-sample.l] Move footstep action client utility to util.l and rename functions.
* [jsk_footstep_controller] Use plane_projection in footstep_makrer
* [jsk_Footstep_controller] floor detection does not need x/y filter
* [jsk_footstep_controller] Use plane_projection when overwriting go-pos goal
* [jsk_footstep_planner] Add option to wait to subscribe plane topic
* [jsk_footstep_controller] Add plane_projection option, which projects footsteps onto subscribed planes
* [jsk_footstep_controller] Add use_snapshot option for detected floor coefficients topic
* [jsk_footstep_controller] Modified stabilizer_watcher's
* [jsk_footstep_controller] fix go-pos-server.l
* [jsk_footstep_controller] Rename old :is-walking method, which is used for overwrite decision
* [jsk_footstep_controller] Use feedback from go-pos-server to get go-pos-server status from client
* [jsk_footstep_controller] Publish feedback just after planning is failed
* [jsk_footstep_controller] add go-pos-server/client
* [jaxon-footstep-controller.l] move util functions to util.l and refactor functions
* [jsk_footstep_controller] remove temporary file
* [jsk_footstep_controller] Add height_offset param to consider crane and so on
* [jsk_footstep_controller] print name of foot
* [jsk_footstep_controller] print messages when collide footsteps
* [jsk_footstep_controller] Modify footstep collision avoidance in jaxon-footstep-controller.l
* [jsk_footstep_controller] Add instant collision avoidance for refined footsteps
* [jsk_footstep_planner] update footstep_marker for appending footsteps continuously
* [jsk_footstep_controller, footcoords] fix for using waitForSensorFrameTransformation
* [jsk_footstep_planner] Add simple launch file to extract floor and estimate its parameters
* [jsk_footstep_controller] new jsk_recognition_utils::Plane() causes eigen alignment issues in 32bit environment
* [jsk_footstep_planner] project odom_init to detected floor plane (default is z = 0)
* [jsk_footstep_controller] Add footstep-controller euslisp script for jaxon
* Contributors: Iori Kumagai, Shunichi Nozawa, Yohei Kakiuchi, Juntaro Tamura

0.1.9 (2016-03-23)
------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_footstep_controller] Add footstep-controller euslisp script for jaxon
* [jsk_footstep_controller] Publish robot_bbox only when analysis_level is whole_links
* [footstep_visualizer] Call st only once
* [footstep_visualizer] Support dynamic_reconfigure API
  to toggle which point to draw.
  And fix some error about tf conversion and nan computation
* [jsk_footstep_controller] Wait for st service to be available
* [jsk_footstep_controller] Ignore error in footstep_visualizer
* [jsk_footstep_controller] Add use_footcoords option to select whether footcoords is used or not
* [jsk_robot_startup] Forgot timestamp
* [jsk_footstep_controller] Publish odom_init as transform and pose_stamped when odom_init is updated
* [jsk_footstep_controller/footstep_visualizer] get leg_vertices from StabilizerService param
* [jsk_footstep_controller] Fix indent of stabilizer_watcher.py
* [jsk_footstep_controller/stabilizer_watcher] Do not call service call to hrpsys,
  use /act_contact_states topic to detect st status
* [jsk_footstep_controller/footstep_visualizer] Fix zmp location by taking
  into account z componrnt.
  closes #518
* [jsk_footstep_planner/footstep_visualizer] Fix indent
* [jsk_footstep_controller] Draw Convex Hull of Support Polygon
* [jsk_footstep_controller/footstep_visualizer] Add Reference capture point and Actual capture point
  and draw text
* [jsk_footstep_planner] Fix skip_cropping=true behavior
* Contributors: Kei Okada, Ryohei Ueda, Yuta Kojio, Iori Kumagai

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* [jsk_footstep_controller] Fix typo: init_odom -> odom_init
* [jsk_footstep_controller] Updated README.md for init_odom
* [jsk_footstep_controller] Fix model file loading
* [jsk_footstep_controller] Add invert_tf option to broadcast odom_init as parent of odom
* not call tf if tilt is absent
* [jsk_footstep_controller] Normalize torque with max-joint-torque and use
  squared norm to be propotional to temperature
* [jsk_footstep_controller] Add sample launch file for root-height.l
* [jsk_footstep_controller] Publish plotting data when computing root height
* [jsk_footstep_controller] Publish /odom_init_trigger when robot stands
  on the ground at the first frame
* [jsk_footstep_controller] Compute root-link height according to torque
  and manipulability. Original version is implemented by Masaki Murooka
  and interface of function is modified to use as library.
* fix function name. weight -> root
* change footcoords param use_imu->false
* [jsk_footstep_planner] Add start-abc button for planner gui using with simulator
* [jsk_footstep_controller] Cleanup and omit a lot of features of footstep controller and confirmed with
  hrpsys/gazebo simulation
* [jsk_footstep_controller/footcoords] Add ~use_imu and ~use_imu_yaw to take
  into account orientation from IMU as well as translation of /odom
* [jsk_footstep_controller] Say something when robot stands on the ground
* [jsk_footstep_planner, controller] Add rviz GUI set for playing with footstep planner
* [jsk_footstep_controller] Launch stabilizer_watcher on HRP2 and JAXON
* [jsk_footstep_controller/footcoords] Use correct timestamp for zmp tf frame
* [jsk_footstep_controller/footcoords] Publish zmp as tf for visualization.
  DO NOT USE THIS FRAME FOR PERCEPTION AND PLANNING because the timestamp is not reliable
* [jsk_footstep_controller/footcoords] Add odom_init frame which holds the pose when robot is put on the ground
* [jsk_footstep_controller/footcoords] Publish body_on_odom frame, which should be useful to represent
  sensordate in "Robot-centric-perspective"
* [jsk_footstep_controller] Add simple-footstep-controller as the most simplest footstep controller using
  :set-foot-steps
* [jsk_footstep_controller/footcoords] Remove odom_root frame
* [jsk_control/footcoords] Use lfsensor and rfsensor
* [jsk_footstep_controller] Add odometry estimation based on leg kinematics.
  Three types of naive algorithm are implemented:
  1) Estimate support leg from force sensors and keep support leg during double stance phase
  2) Estimate support leg from force sensors and change support leg during double stance phase by leg forces
  3) Estimate support leg from force sensors and change support leg during double stance phase by zmp
* [jsk_footstep_controller] Remove catkin.cmake and use CMakeLists.txt only
* [jsk_footstep_controller] Publish synchronized forces from foot_coords and
  subscribe it from foot_coords internally.
  Update alpha (low pass filter parameter) to 0.1 from 0.5.
  Update queu length not to drop messages.
* [jsk_footstep_controller] Update parmeter files about footstep configuration
* [jsk_footstep_controller] Add script to generate footstep parameter from
  euslisp models
* [jsk_footstep_controller/footstep_visualizer] Visualize zmp
* [jsk_footstep_planner, jsk_footstep_controller] Support HRP2JSKNT
* [jsk_footstep_planner, jsk_footstep_controller] Add USE_JOY option
* [jsk_footstep_planner, jsk_footstep_controller] Refactor launch file and
  add no_recog.launch
* [jsk_footstep_controller] Move robot-boundingbox.l from drc_task_common
* [jsk_footstep_controller/footstep_visualizer] Reverse position of left
  and right
* [jsk_footstep_controller/footstep_visualizer] Use BGRA8 to represent
  footstep location and COP position
* [jsk_footstep_controller] Add new script to visualize cop of each leg
* [jsk_footstep_controller] Add script to dump mocap output
* Contributors: MasakiMurooka, Ryohei Ueda, Yu Ohara, Iori Kumagai

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
