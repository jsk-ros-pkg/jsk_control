^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_ik_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
