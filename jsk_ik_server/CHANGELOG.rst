^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_ik_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.18 (2024-12-09)
-------------------

0.1.17 (2023-05-28)
-------------------

0.1.16 (2022-10-27)
-------------------
* [jsk_ik_server] add TABLIS reachability map (`#777 <https://github.com/jsk-ros-pkg/jsk_control/issues/777>`_)
* https://projects.coin-or.org have mved to github (`#776 <https://github.com/jsk-ros-pkg/jsk_control/issues/776>`_)
* [jsk_ik_server] add fetch ik reachability map script (`#769 <https://github.com/jsk-ros-pkg/jsk_control/issues/769>`_)
* avoid zero division ik ik-evaluation.l (`#767 <https://github.com/jsk-ros-pkg/jsk_control/issues/767>`_)
* [jsk_ik_server] add baxter reachability map script (`#771 <https://github.com/jsk-ros-pkg/jsk_control/issues/771>`_)

  * add baxter ik reachability map images in readme
  * add fetch ik reachability map in readme
  * add baxter reachability map

* fix for test (`#773 <https://github.com/jsk-ros-pkg/jsk_control/issues/773>`_)

  * indigo requres to add find_pacakge(roseus) to compile roseus message
  * remove roseus from find_package(catkin at jsk_footstep_planner and jsk_ik_server

* [jsk_ik_server] fix typo in hrp2.sh and pr2.sh (`#768 <https://github.com/jsk-ros-pkg/jsk_control/issues/768>`_)
* [jsk_ik_server] add readme for jsk_ik_server (`#772 <https://github.com/jsk-ros-pkg/jsk_control/issues/772>`_)
* 2to3 -w -fprint . (`#763 <https://github.com/jsk-ros-pkg/jsk_control/issues/763>`_)

* Contributors: Kei Okada, Shingo Kitagawa

0.1.15 (2018-05-16)
-------------------

0.1.14 (2018-01-15)
-------------------

0.1.13 (2017-04-18)
-------------------

0.1.12 (2017-02-22)
-------------------

0.1.11 (2017-02-09)
-------------------

0.1.10 (2016-12-15)
-------------------
* Add centroid method to ik-grid (`#636 <https://github.com/jsk-ros-pkg/jsk_control/pull/636>`_)
  * [wholebody_manipulation_planner] add :centroid method to ik-gird.
  * [jsk_ik_server/euslisp/ik-evaluation.l] fix indent.
* [jsk_ik_server/euslisp/ik-evaluation.l] add fullbody and initial-pose arguments to ik-evaluation. (`#602 <https://github.com/jsk-ros-pkg/jsk_control/pull/602>`_)
* [jsk_ik_server/euslisp] extend IK grid function (`#576 <https://github.com/jsk-ros-pkg/jsk_control/pull/576>`_)
  * [jsk_ik_server/euslisp/ik-evaluation.l] add move-target and orient-centerp arguments to ik-evaluation function.
  * [jsk_ik_server/euslisp/ik-evaluation.l] add :insidep method to ik-grid class.
  * [jsk_ik_server/euslisp/ik-evaluation.l] save cube instance in :cube method.
* Contributors: Masaki Murooka

0.1.9 (2016-03-23)
------------------

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* [jsk_ik_server] Remove grids which are too near to robot
  in stand location planning
* [jsk_ik_server] Add script to generate reachability images for pr2 and hrp2
* [jsk_ik_server] Implement Brute-force stand location search.
* [jsk_ik_server] Implement stand location planning with taking into
  account range of targets
* [jsk_ik_server] Visualize ik-grid by irtviewer
* [jsk_ik_server] Implement stand location planning based on continuous ik
  grid.
  Currently only position and mean of pdf is taken into account
* [jsk_ik_server] Fix jaxon_ik_evaluation.md to render properly on github
* [jsk_ik_server] Add markdown to visualize ik-grid evaluate
* [jsk_ik_server] Pretty-printing of progress of generating ik grid like
  min-max table generation
* [jsk_ik_server/plot_ik_grid.py] Add second argumment to save image file
  instad of showing plot on GUI
* [jsk_ik_server] Script to visualize reachability in heatmap manner
* [jsk_ik_server] Add ik-evaluation.l to evaluate spacial ik quarity
* Remove manifest.xml and Makefile and use catkin style filesystem
* Contributors: Ryohei Ueda

0.1.6 (2015-06-11)
------------------
* add files for jaxonred
* add jaxon to ik_server
* change order in find_pkg

0.1.5 (2015-01-08)
------------------
* pass warnp as argument in :fullbody-ik-main
* remove sample robot test
* Merge branch 'master' into update-samplerobot-in-ik-server
* use unix:usleep because hrpsys stops clock
* use hrpsys sample robot for ik server
* Contributors: JSK Lab Member, leus, Yusuke Furuta

0.1.4 (2014-10-21)
------------------
* support hrp2jsknts in ik-server
* adding hrp2w-ik-server.l

0.1.3 (2014-10-10)
------------------
* add ik-server wait function, in travis, ik-server is slow to start
* install euslisp in share directory
* change ik-server files all exectable
* add :ik-server-name args, smaple robot has name with - charactre
* forbit to use copy-object of link cascoords object

0.1.2 (2014-09-08)
------------------

0.1.1 (2014-09-04)
------------------
* use singleton class to maintain view point of rviz to have persistency
  across several plugins
* support multiple instances per one plugin class
* Merge pull request `#47 <https://github.com/jsk-ros-pkg/jsk_control/issues/47>`_ from s-noda/ik_server_publish_default_end_cords
  ik server publishes posestamped of default-end-coords
* add publish statement of default end coords as posestameped message, frame_id = root-link
* remove needless comments
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_control into ik-server-remote-unused-overwrite
* support jsk_teleop_joy in robot-controller-sample.launch of jsk_ik_server
* remove needless ik-server class definitions
* support other robot, not only staro in robot-controller-sample.launch
* forbit to generate viewer if display missing
* add dependacy on rostes
* remove needless ik-server impl
* add test files and rostest declearation in catkin,cmake and CMakeLists.txt
* remove :cog-convergence-check, :inverse-kinemtiacs-with-error functions
* add sample dir, using spacenav and ik-server, control robot
* add IK_OPTION argument to staro-ik-server-test.launch
* add staro-ik-server.launch
* add IK_OPTION argument to hrp2jsk-ik-server-test.launch
* add additional-ik-options
* added staro specific code for ik-server
* fix usage of cog-convergence-check function, correspond to irtmode.l update
* `#9 <https://github.com/jsk-ros-pkg/jsk_control/issues/9>`_: install moveit_msgs with package.xml for jsk_ik_server
* fix udpate-support-links timing, please call this fucntion after initialization of ik-server
* hand existance check add
* add default-end-coords variable, forbit to use :end-coords statement
* add client_test_with_leg flag, check if support polygon correctly transformed
* fix transformation of support links, convert support polygon  to the target coordinate
* fix supprot-link usage, and remove :end-coords
* bug fix, use link name as frame_id
* add configuration dir, but now, not supported yet
* supprot group_name=whole_body, fix-limb='(:rleg :lleg)
* /odom transformation validated without tf
* ik-server transform all coords using robot model and from-id,
* fix name -> link matching, use find-link-from-name funciton
* convert all frame_id to root-link-frame-id slots, if null, convert using robot model
* fix for collision check, add slot variable of defualt collisoin link
* fix the timing of make-convex function, just before call-ik-server
* add some parameter for collision avoidance
* :ik-server-call function support collision-avoidance-link-pair
* all-test.launch add, for test
* added launch/hrp2jsknt-ik-server.launch
* added svn exclude in installation of jsk_ik_server/catkin.cmake
* fix ik-server return joint_State, link names -> joint names
* joint-state message methods check fix, for hydro
* moveit_msgs::MoveItErrorCodes::*NO_IK_SOLUTION* check fix
* bound check for hydro message type change
* assoc hrp2jsknt model hand and wrist
* add link-list arguments, hrp2 model separate into body and hands
* pr2 has no leg limbs
* add some comment, and test programs are changed to use :fix-limbs option
* fix robot link-list slots variable, pr2 had not had gripper links
* remap /solve_ik -> //solve
* fix typo, transfrom -> transform
* multi_dof_joint_States :joint_transforms -> :transform in hydro
* hrp2 ik-server files donot use tf
* multi-6dof-joint-states supported,
* remove viewer arg from :update-joint-states
* comment quaternion usage
* base coords in joint_states, eular angle and quaternion supported
* add slots value ik-server-name and ik-server-service-name to set node name and service name
* add ik-server-call function, this functions can be used just the same as euslisp :fullbody-inverse-kinematics functions
* mv fullbody-ik-client-test.l to test dir and fix some dependancy of test launcher files. please check test launcher files before change configuration
* remove unused require statement
* remove test dependancy from manifest.xml, it's ok? to remove pr2eus and atlashogehoge
* catkinize jsk_ik_server
* make fullbody ik client class for ik server
* add :support-links args, change foot-convex and targe-centroid-pos
* remove unused comment, and some arg name fix
* :fix-limbs '(:limb1 :limb2 ....) supported
* simplyfy :fullbody-ik-main, old versino move to old-ik-server.k
* load only robot model file instead of interface file.
* remove fix-limb-cords slots,
* change ik-server-test.launch for fullbody-ik-client.l
* hrp2jsk-test fucntino add
* change dir configuration, each ik-server.l move to ik-server-impl dir
* add :inverse-kinematics function, causion, to fix pr2 model torso, :torso-fix t :use-torso 0 option needed
* fix ik-server-call function, options has nil list supported
* fix objects usage
* add some test functions
* rename *hoge* slot variable to hoge
* rename eus-fullbody-ik-ex -> ik-server-util, i think -ex is terrible naming
* remove unused functions
* add old-ik-server, from hrpsys_gazebo_atlas
* change order m -> mm
* joint name convert to string, and robot-model -> cascaded-link
* change euscollada-robot -> robot-model
* add viewer slots in ik-server class, not only irtviewr, but pickview can be used
* remove global variables, usage, generate robot object, and call (ik-server-call :robot )
* move-target, taget-coords, links-list length check add
* change some comment, not so important
* overwrite make-convex function, bacause hrp2 has toe joint
* centroid < convex check add
* additional-weight-list supported,
* bug fix, if target-centrid-pos == null, then not call cog-ceonvergence check
* add base coords to return statement of ik-server
* arrow object in ik-server viewer trach the first coordinamte of target ones
* debug-view flag can be changed
* ik-sever.l validated with fullbody-ik-client.l, but there is a strange change, base link tranformation need to be called twice?(line: 270)
* fullbody-ik-cline.l add, call ik-server with the same argment of euslisp :fullbody-inverse-kinematics functino
* coordinates fix
* fix some key name of ik_request
* add fullbody-inverse-kinematics-service-cb functino, for group_name =:fullbody-inverse-kinematics, not tested
* do not load robot-interface.l , load just model.l
* added hrp2 launch files
* deleted atlas-eus-ik-client.l
* remove arm_navigation_msgs
* add more debug messages
* not load pr2-interface.l, just load model files.
* reverted last commit. added hrp2jsk, hrp2jsknt server programs.
* merge pr2 and atlas ik server
* deleted atlas-eus-ik-client.l : client program is common for all robots.
* use make-foot-convex for humanoid robot
* removed atlas-end-coords.l: this is copy of the file under hrpsys_gazebo_atlas and is not necessary here.
* removed atlas specified files from eus-fullbody-ik-ex.l and ik-server.l
* change fullbody-ik function to class method
* add eus ik server package
* Contributors: Ryohei Ueda, Yohei Kakiuchi, Yusuke Furuta, Kei Okada, Masaki Murooka, Shintaro Noda
