^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_footstep_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2015-06-11)
------------------
* [jsk_footstep_planner] Enable roll paranoid mode
* Merge pull request #327 from garaemon/set-heuristic
  [jsk_footstep_planner] Add service interface to set heursitic function
* [jsk_footstep_planner] Add service interface to set heursitic function
* [jsk_footstep_controller] Fix parameters for jaxon red
* [jsk_footstep_planner] Fix for terrain task
* [jsk_footstep_planner] Add topic interface to project footprint
* [jsk_footstep_planner] Support JAXON_RED
* [jsk_footstep_planner, jsk_footstep_controller] Update for las vegas terrain with jaxon
* [jsk_footstep_planner] Enable roll paranoid mode again
* [jsk_footstep_planner, jsk_footstep_controller] Update for jaxon terrain
* [jsk_footstep_planner] Add simple script to transform frame_id of
  jsk_footstep_msgs/FootstepArray
* [jsk_footstep_controller] Change gait generator parameters according to
  plane condition including pitch angle and taking into account
  if transition is upward or downward
* [jsk_footstep_planner, jsk_footstep_controller] Compute x and y from
  previous coordinates to detect rolled plane
* [jsk_footstep_planner, jsk_footstep_controller] Support rolling terrain,
  I hope
* [jsk_footstep_planner, jsk_footstep_controller] Fix typo and update
  parameter for terrain
* [jsk_footstep_planner, jsk_footstep_controller] Support jaxon parameter files
* [jsk_footstep_planner, jsk_footstep_controller] Support jaxon footstep planning, Do not merge yet
* [jsk_footstep_planner] locally search reasonable goal when snapping
* [jsk_footstep_planner] Update offset parameter
* [jsk_footstep_planner] Support offset parameter from end coords to
  center of foot polygon
* [jsk_footstep_planner] Optimize function to remove shadow cells by using
  local coordinates
* [jsk_footstep_planner] Update projection parameter for slope terrain
* [jsk_footstep_planner] Add ~remove_shadow_cells to add padding to shadow cells
* [jsk_footstep_planner] Improve planning for different levels:
  1. Move successors a little bit when projecting footprint to different
  level
  2. Use footstep coordinate rather than mid-coords of the footstep as
  goal coordinates
* [jsk_footstep_planner] Check transition limit when planning across
  diffrent levels
* [jsk_footstep_planner] Add *gui-debug* symbol and ~toggle_gui_debug
  service to toggle debug using gui
* [jsk_footstep_planner] Add service API to project pose onto the nearest grid
* [jsk_footstep_planner] More greedy heuristic and compile euslisp code correctly
* [jsk_footstep_planner] Separate successors for same level and transition
  across different level
* [jsk_footstep_planner] Update successors' parameters
* [jsk_footstep_planner] Check range of grids first in occupancy-grid
* [jsk_footstep_planner] Use glVertexPointer and glDrawArrays to draw occupancy-grid
* [jsk_footstep_planner] Optimize creation of occupancy-grid by using
  integer-vector rather than hash-table
* [jsk_footstep_planner] Fix bug to compute rotate footstep to snap onto planes
* [jsk_footstep_planner] Update successors parameter
* [jsk_footstep_planner] add geo package to quaternion-from-two-vectors
* [jsk_footstep_planner] Read footstep parameter from file rather than ros parameter
* [jsk_footstep_planner] Use quaternion-from-two-vectors to compute rotate
  in project-coords-on-to-plane
* [jsk_footstep_planner] Add method to generate occupancy-grid from face
* [jsk_footstep_planner] Use mtimer instead of ros::time-now
* [jsk_footstep_planner] Fix minor bugs
* [jsk_footstep_planner] Memoize projecting grid and re-use it when the
  planner checks the plane is placable and refactor function names
* [jsk_footstep_planner] Optimize hash size according to cell num
* [jsk_footstep_planner] A lot of improvements on planner
  * Fix about orientation of projected footstep
  * Lazy evaluation to check if footprint is able to be on grid
  * Use mid-coords of footprints to evaluate heuristic
* [jsk_footstep_planner] Do not check if the footstep can be placable on
  plane across planes
* [jsk_footstep_planner] Support SimpleOccupancyGrid in footstep planner
* [jsk_footstep_planner] Snap geometry_msgs::PoseStamped (from rviz) onto grid map
* [jsk_footstep_planner] Move more functions to footstep_planner_utill.l
  from footstep_planner.l
* [jsk_footstep_planner] Support color in occupancy-grid class
* [jsk_footstep_planner] Return vertices in global coordinates in
  :vertices method of occupancy-grid
* [jsk_footstep_planner] Euslisp binding of
  jsk_recognition_msgs::SimpleOccupancyGrid message
* [jsk_footstep_planner] separate standalone utility functions into footstep_planner_util.l
* [jsk_footstep_planner] Add publisher of polygon of footprint for HRP2JSK
* [jsk_footstep_planner] Add euslisp wrapper to snap footstep on planes
* [jsk_footstep_plannar] Resolve pose of footstep respacted to initial footstep
* [jsk_footstep_plannner] Visualize euslisp footstep on rviz
* [jsk_footstep_planner] Use jsk_recognition_msgs
* Contributors: Ryohei Ueda, Yu Ohara

0.1.5 (2015-01-08)
------------------
* renamed make_sumple function
* added make-coords-list function
* added inverse_reachablity_with_given_coords
* Update drcmodel for current planner
* Add sample to compare heuristic functions
* add api to change successor
* Merge remote-tracking branch 'origin/master' into add-breakpoint-text
  Conflicts:
  jsk_footstep_controller/euslisp/footstep-controller.l
  jsk_footstep_controller/launch/hrp2jsknt_real_full.launch
* Add text publishing when checking breakpoint
* Do not allow step over 250mm stride
* Supress x-transition after z-transition. All the threshold is hard-coded
* Update footstep parameter for climing up stairs:
  larger footstep and smaller footprint
* Add dimensions of footsteps to the result of footstep planner
* Visualize footstep successors
* roseus only needs runtime
* Contributors: Kei Okada, Ryohei Ueda, Yu Ohara

0.1.4 (2014-10-21)
------------------

0.1.3 (2014-10-10)
------------------

0.1.2 (2014-09-08)
------------------

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
