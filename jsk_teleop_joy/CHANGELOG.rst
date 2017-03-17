^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_teleop_joy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.12 (2017-02-22)
-------------------

0.1.11 (2017-02-09)
-------------------

0.1.10 (2016-12-15)
-------------------
* [jsk_teleop_joy] package.xml : add pygame run depend (`#657 <https://github.com/jsk-ros-pkg/jsk_control/pull/657>`_)
* Add toggle footstep marker mode joy interface (`#607 <https://github.com/jsk-ros-pkg/jsk_control/pull/607>`_)

  * src/jsk_teleop_joy/plugin/joy_footstep_marker.py: Add joy interface to toggle planning mode

* Add plugin to send cmd_vel from joystick contorller (`#600 <https://github.com/jsk-ros-pkg/jsk_control/pull/600>`_)

  * [jsk_teleop_joy] src/jsk_teleop_joy/joy.py : Add plugin_package option to JoyManager to use joy plugins in other packages
  * [jsk_teleop_joy] src/jsk_teleop_joy/plugin/joy_cmd_vel.py : Add orthogonal_axis_mode, which does not allow diagonal movement
  * [jsk_teleop_joy] src/jsk_teleop_joy/plugin/joy_cmd_vel.py : Add plugin to send cmd_vel from joystick contorller

* Add utility tools for jaxon footstep planner (`#598 <https://github.com/jsk-ros-pkg/jsk_control/pull/598>`_)

  * [jsk_footstep_planner] launch/joy_footstep_marker.launch : Add joy_footstep_marker launch, which control footstep_marker from joystick controller

* Project odom_init to detected floor (`#579 <https://github.com/jsk-ros-pkg/jsk_control/pull/579>`_)

  * [jsk_teleop_joy] Check result of service call by exception handling
  * [jsk_teleop_joy] Display OverlayMenu before execute footstep
  * [jsk_teleop_joy] sync pre_pose in joy_footstep_marker only at first of the pose update sequence
  * [jsk_teleop_joy] Add marker_name arg to getCurrentMarkerPose and initialize marker by initial_footstep_marker in reset process
  * [jsk_teleop_joy] Add footstep marker synchonization to joy_footstep_marker
  * [jsk_teleop_joy] Add joycontroller interface plugin for footstep_marker

* Contributors: Kanae Kochigami, Iory Kumagai

0.1.9 (2016-03-23)
------------------

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* [jsk_teleop_joy/joy_relative_converter.py] add reset command
* [jsk_teleop_joy/scripts/joy_relative_converter.py] fix bug to support buttons
* Revert "[jsk_teleop_joy/scripts/joy_relative_converter.py] fix bug to support buttons"
  This reverts commit 1704b24d2b96aae962e4c87968f68078442417a2.
* [jsk_teleop_joy/scripts/joy_relative_converter.py] fix bug to support buttons
* put most process into class method
  Conflicts:
  jsk_teleop_joy/scripts/joy_relative_converter.py
* instantiate before subscribe
  Conflicts:
  jsk_teleop_joy/scripts/joy_relative_converter.py
* fix bug around page-change
* implement page-change
* add joy_relative_converter
* [jsk_footstep_controller, jsk_teleop_joy] Use footstep-controller.l and lock/unlock furutaractive
  model during exeucuting footsteps
* Remove manifest.xml and Makefile and use catkin style filesystem
* Contributors: Ryohei Ueda, Satoshi Iwaishi

0.1.6 (2015-06-11)
------------------
* [drc_task_common] Modify threshold of brake: 0.9 -> 0.5
* [jsk_teleop_joy] Subscribe execute flag and disable update command when execute flag is false
* [jsk_teleop_joy] Call wait_for_message only once in synchronize
* [jsk_teleop_joy] synchronizeAllCommand do not takes argument
* [jsk_teleop_joy] Support single synchroniation
* [jsk_teleop_joy] Fix neck-p/y joint command range
* [jsk_teleop_joy] Add initialization service to vehicle joy
* [jsk_teleop_joy] Add synchronize method to vehicle plugin to prevent overwrite previous command when respown
* [jsk_teleop_joy] Remove set_current_step_as_max functions because they are moved to vehicle_ui
* [jsk_teleop_joy] Add neck-p interface to joystick controller for vehicle task
* [jsk_teleop_joy] Speed down handling command: 0.05->0.025
* [jsk_teleop_joy] Do not set 0.0 command as max step
* [jsk_teleop_joy] Add set_current_step_as_max function to vehicle_ui
* [jsk_teleop_joy] Modify teleop command in joystick controller for vehicle task
* [jsk_teleop_joy] Rename vehicle.launch to joy_vehicle.launch
* [jsk_teleop_joy] Add neck_angle_max valiable
* [jsk_teleop_joy] Add functions for look around to vehicle.launch
* Add brake command and modify accel command specification
* Add arguments to determine joystick dev path and namespace for ocs
* [jsk_teleop_joy] Adjsut command publish rate for vehicle
* [jsk_teleop_joy] Adjsut handle resolution and modify to publish topic constantly
* [jsk_teleop_joy] Add joystick program for vehicle
* remove DEV argument because it was removed from robot_trackball_head.launch

0.1.5 (2015-01-08)
------------------
* [joy_mouse] Use name of kensington mouse and remove dev file
  specification.
  [jsk_teleop_joy] Remove DEV argument
* add script to publish pose stamped with spacenav
* Fix poping-up cancel window by broadcasting canceled information
* add api to change successor
* Add text publishing when checking breakpoint
* Publish usage of footstep planner joy
* disable/enable head control with trackball buttons, move head joint continuously.
* Contributors: Masaki Murooka, Ryohei Ueda, Yusuke Furuta

0.1.4 (2014-10-21)
------------------
* Merge pull request #112 from mmurooka/overwrite-write-command-in-midi-player
  Overwrite writing command in midi_config_player.py
* overwrite writing command in midi_config_player.py
* add pr2_relay.launch
* publish joy topic only when midi state is changed.
* add feedback config to b-control.yaml

0.1.3 (2014-10-10)
------------------
* add b_control_status.py
* add config file of b-control
* Add joystick interface for jsk_pcl_ros/EnvironmentPlaneModeling
* use scripts/head_control_by_trackball.py for general robot. implimented launch file for pr2 and hrp2
* Merge branch 'master' into select-menu-with-analog-stick
* autorepeat joy input
* check analog input
* remap tf
* test analog check
* get argument  for set pose

0.1.2 (2014-09-08)
------------------

0.1.1 (2014-09-04)
------------------
* remap joint states and DEV
* add script to control head via trackball
* remove trackpoint_joy.py
* mvoe python scripts to parse state to src directory
* fix bag at first time
* update menu
* publish at 10hz
* set autorepeat rate
* use joy mux
* make JoyStatus class
* fix the menu when walking is canceled and update the pose from joy stick according
  to the snapped pose availble by marker
* pop menu when cancel the footstep and support resuming from joystick
* cancel walking via joystick
* update diagnostics information about footstep planning and joy stick stuff
* compile euslisp file before running footstep planner
* circle button to move arm
* Merge branch 'master' into fix-jsk-interactive-marker-plugin
  Conflicts:
  jsk_teleop_joy/launch/pr2.launch
* * remove jsk_interactive_marker.launch and integrate it to pr2.launch
  * rewrite jsk_interactive_marker plugin to modern plugin style
* add plugin to show usage
* delete empty lines
* Merge branch 'master' into add-plugin-for-jsk-interactive-marker
  Conflicts:
  jsk_teleop_joy/manifest.xml
  jsk_teleop_joy/package.xml
  jsk_teleop_joy/src/jsk_teleop_joy/plugin/joy_pose_6d.py
* modify launch file
* jsk_teleop_joy depends on jsk_rviz_plugins
* update moveit teleop plugin to the latest change of moveit
* add center button to JoyStatus class and use center button
  to choose menu
* add new plugin to relay and convert joy message to ps3
* add Relay plugin to jsk_teleop_joy
* use singleton class to maintain view point of rviz to have persistency
  across several plugins
* show overlay menu on rviz to swtich plugins
* support multiple instances per one plugin class
* use diagnostic_updater package to generate diagnostic messages
  rather than publish diagnostic_msgs directly
* optimize rviz animation smoother by joy stick controller
* support jsk_teleop_joy in robot-controller-sample.launch of jsk_ik_server
* publish the status of jsk_teleop_joy to /diagnostics.
  decrease the number of the messages if the joy stick type is failed to
  be estimated.
  publish the status of the estimation to /diagnostics
* make interactive_midi_config available for hydro
* fix midi_config_player for groovy
* make midi_config_player available for hydro
* add button to control interactive marker
* transform PoseStamped when setting marker pose
* add method to set pose
* add method to change move arm
* use triangle button to send menu
* fix find -> find_module to detect catkin or rosbuild
* add config for padcontrol
* support groovy on all the plugins
* use imp module to decide use load_manifest or not
* send 'move' when circle button is pushed
* add dependancy on jsk_interactive_marker
* rename plugin scripts to avoid msg import bug
* add import statement
* use load_manifest on groovy
* use load_manifest on groovy
* add end effector controller interface
* JoyFootstepPlanner: publish execute if circle button is pushed
* JoyFootstepPlanner: reset goal pose if cross button is pushed
* determines the initial position of goal according to the specified frame_id and offset for the legs
* add tf_ext.py to jsk_teleop_joy. it's a set of utitlity function for tf
* revert to use depend tag for view_controller_msgs
* write about select button
* write about how to implement plugin
* write about how to export the plugins
* update docs
* use upper case for MIDI
* add list of plugins
* update docs
* update docs
* add link to each script
* update some docs
* add document about `midi_write.py`
* add movie of interactive configuretion of midi device
* use english in README.md#interactive_midi_config.py
* fix style of ordered list
* `#2 <https://github.com/jsk-ros-pkg/jsk_control/issues/2>`_: automatically detect the game controller type at joy_footstep.launch
  use type=auto parameter
* `#2 <https://github.com/jsk-ros-pkg/jsk_control/issues/2>`_: rename xbox.launch and xbox_footstep.launch to joy.launch and joy_footsetp.launch.
  it support many game controllers now and the name did not match the current state.
* `#2 <https://github.com/jsk-ros-pkg/jsk_control/issues/2>`_: detect ps3 wireless automatically
* `#2 <https://github.com/jsk-ros-pkg/jsk_control/issues/2>`_: use auto mode as default
* `#2 <https://github.com/jsk-ros-pkg/jsk_control/issues/2>`_: update document about ps3 bluetooth
* mv jsk_joy/ jsk_teleop_joy/
* rename jsk_joy -> jsk_telop_joy
* fix to use rosdep
* adding footstep planning demo plugin
* updating the parameters
* arg1 = topic name, arg2 = device name
* fix topic name
* install subdirectory into dist_package
* auto detecting xbox/ps3wired
* use joy_main as a wrapper of jsk_joy python library
* not use roslib.load_manifest if the distro is hydro
* installing launch file and so on
* catkinized jsk_joy package
* changed frame from base_link to odom
* added JoyGoPos for plugin.xml
* added gopos.py for teleoperation locomotion command
* added gopos.launch for teleoperation locomotion command
* sample launch for marker_6dof
* tuned parameters to move camera
* adding moveit plugin for controlling moveit from gaming controllers
* launch file for pr2 moveit
* adding README
* adding configuration for launchpad mini
* adding output configuration to QuNeo
* supporting output of MIDI
* adding script to test output of midi devices
* mapping buttons automatically from axes
* update midi configuration
* script to verbose midi input
* not printing input
* adding nanokontrol2.yaml
* updating configuration file
* supporting 144/128 key event
* adding config file for icontrols pro
* adding scripts to configure midi device interactively
* changing joy footstep planner plugin to use footstep marker in jsk_interactive_marker
* adding interface to call footstep planning from game controllers
* adding verbose plugin for debugging and support wired ps3 controller
* add nanopad2_joy.py, touchpad and scene button supported
* adding sample to run xbox footstep plugin
* update orientation way to local
* supporting local z movement acoording to orientation
* adding manual footstep generator interface
* updating parameters of view rotation
* test pulibhs program for joystick, any joystick ok?
* supports to toggle follow view mode
* devided trackpoint joy publisher and status class to two files.
* added nanopad_joy.py nanopad_status.py for KORG nanoPAD2
* updating some parameters
* supporting pitch and roll
* implementing jsk_joy as plugin system
* changed class name of nanokontrol status: NanoKONTROL2 -> NanoKONTROL2Status
* add nanokontrol_status.py. convert data from Joy msg to nanoKONTROL class instance.
* support touchpad; auto-detect device id
* light turns on when button is pushed
* added device link URL of vestax_spin2
* added trackpoint_joy.py. publish thinkpad trackpoint status as Joy.
* bugfix button type
* set vestax_spin2.py execuable
* chnaged button index of akailpd8. set for PROG1 PAD mode.
* bugfix indent
* added URL of device web page for lanchpad
* add controller for vestax spin 2
* added akaiLPD8.py
* added device URL link for nanokontrol
* add script to publish joy_message with launchpad mini
* deleted debug outpu in nanokontrol_joy.py
* add rosdep name=python
* added nanokontrol_joy.py for publishing nanoKONTROL2 input as Joy.
* update some parameters
* update some parameters
* using left analog to zoom in/out
* introducing new package: jsk_joy
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda, Satoshi Iwaishi, Yuki Furuta, Yusuke Furuta, Shunichi Nozawa, Shintaro Noda, Youhei Kakiuchi
