^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_qp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.16 (2022-10-27)
-------------------
* check x::*display* is available before running graph-view (gnuplot) (`#776 <https://github.com/jsk-ros-pkg/jsk_control/issues/776>`_)
* ptmotiongen: no visualization / print if argument is nil. (`#729 <https://github.com/jsk-ros-pkg/jsk_control/issues/729>`_)
* [optmotiongen] inverse-kinematics for the discrete target (`#728 <https://github.com/jsk-ros-pkg/jsk_control/issues/728>`_)

  * optmotiongen: add test of discrete-kinematics-configuration-task.
  * optmotiongen: update manual.pdf
  * optmotiongen: add manual for discrete target ik.
  * optmotiongen: add sample/sample-sqp-optimization-discrete-kinematics.l
  * optmotiongen: add sample function sample-arm-reach-ik-discrete-raw.
  * optmotiongen: add discrete-kinematics-configuration-task.l
  * optmotiongen: flatten for drawing kin coords list for support discrete-kinematics-configuration-task.

* [optmotiongen] add sample to maximize contact force (`#727 <https://github.com/jsk-ros-pkg/jsk_control/issues/727>`_)

  * optmotiongen: update manual.pdf
  * optmotiongen: add demo-jaxon-hand-force.l
  * optmotiongen: add :wrench-maximize-regular-vector to instant-config-task.
  * optmotiongen: add norm-regular-scale-coeff to instant-config-task.
  * optmotiongen: enable to set nil for print-status-interval and update-viewer-interval.
  * optmotiongen: add sqp-convergence-check and sqp-failure-callback.
  * optmotiongen: enable to set external-wrench-list.

* [optmotiongen] visualize force fix (`#722 <https://github.com/jsk-ros-pkg/jsk_control/issues/722>`_)

* fix for test (`#723 <https://github.com/jsk-ros-pkg/jsk_control/issues/723>`_)

  * optmotiongen: fix and reduce some tests because travis does not finish within 50 min.
  * optmotiongen: fix for visualize contact force in sample-sqp-optimization-trajectory.l.
  * optmotiongen: add functions for visualizing force and moment in playing motion

* [eus_qp/optmotiongen] add target-posture-scale-list argument (`#718 <https://github.com/jsk-ros-pkg/jsk_control/issues/718>`_)

  * optmotiongen: add target-posture-scale-list argument to instant-configuration-task :init method. support to change scale for each posture joint.

* add demo rhp4b reach suitcase instant manip (`#715 <https://github.com/jsk-ros-pkg/jsk_control/issues/715>`_)
* [optmotiongen] fix gravity torque calculation (`#714 <https://github.com/jsk-ros-pkg/jsk_control/issues/714>`_)

  * optmotiongen: update manual.pdf for gravity torque document.
  * optmotiongen: consider fixed link weight to calculate gravity torque. ToDo: The effect for centroid is not considered yet.
  * optmotiongen: update gravity-torque-jacobian calculation and document.
  * optmotiongen: add euslisp/demo/demo-rhp4b-torque-gradient.l
  * optmotiongen: test torque calculation with irtdyna result.

* [optmotiongen] Rebase https://github.com/jsk-ros-pkg/jsk_control/pull/711 (`#713 <https://github.com/jsk-ros-pkg/jsk_control/issues/713>`_)

  * optmotiongen: update manual.pdf for torque gradient update.
  * optmotiongen: update torque gradient calculation (get-contact-torque-jacobian function) and update document.
  * optmotiongen: add sample and test of torque gradient.

* set adjacent regular scale as list format via :adjacent-regular-scale-list argument. add sample for that argument. (`#708 <https://github.com/jsk-ros-pkg/jsk_control/issues/708>`_)

  * optmotiongen: add test of min/max angle of root virtual joint from ik wrapper.
  * optmotiongen: support min/max angle of root virtual joint from ik wrapper.

* set adjacent regular scale as list format via :adjacent-regular-scale-list argument. add sample for that argument. (`#708 <https://github.com/jsk-ros-pkg/jsk_control/issues/708>`_)
* [eus_qp/optmotiongen] support min/max angle of root virtual joint (`#709 <https://github.com/jsk-ros-pkg/jsk_control/issues/709>`_)

  * optmotiongen: add test of min/max angle of root virtual joint from ik wrapper.
  * optmotiongen: support min/max angle of root virtual joint from ik wrapper.

* [eus_qp/optmotiongen] dynamic motion generation with bspline (`#705 <https://github.com/jsk-ros-pkg/jsk_control/issues/705>`_)

  * [eus_qp/optmotiongen/manual] update manual.pdf for bspline-dynamic-configuration-task.
  * [eus_qp/optmotiongen/euslisp/contact-kinematics.l] add look-at-contact class and sample, test for that.
  * [eus_qp/optmotiongen] update sample and test for bspline-dynamic-configuration-task.
  * [eus_qp/optmotiongen/euslisp/demo,sample] move from demo to sample directory.
  * [eus_qp/optmotiongen/euslisp/sample,test] add option to supress graph generation for travis test.
  * [eus_qp/CMakeLists.txt] install optmotiongen directories.
  * [eus_qp/optmotiongen/euslisp/demo] rename hrp2 line face sample.
  * [eus_qp/optmotiongen/test] fix test name to use '_' instead of '-'. cf. http://wiki.ros.org/Names
  * [eus_qp/optmotiongen/euslisp/sample] use irtviewer :draw-floor and :floor-color methods instead of direct slots access.
  * [eus_qp/optmotiongen/test,euslisp/sample] add sample and test with irteus sample-robot.
  * [eus_qp/optmotiongen/doc] add README for bspline-dynamic-configuration-task.
  * [eus_qp/optmotiongen/euslisp/bspline-configuration-task.l] support delta-time argument in :generate-angle-vector-sequence.
  * [eus_qp/euslisp/contact-optimization.l] change drawing color from white to yellow for visibility in white background viewer.
  * [eus_qp/optmotiongen/manual] update manual for bspline-dynamic-configuration-task.
  * [eus_qp/optmotiongen] add bspline-dynamic-configuration-task and sample.
  * [eus_qp] add test-load-euslisp-files.

* [eus_qp/optmotiongen] add sqp with multi solution candidates (`#702 <https://github.com/jsk-ros-pkg/jsk_control/issues/702>`_)
* [eus_qp/optmotiongen] Test eus loading without bash script (`#707 <https://github.com/jsk-ros-pkg/jsk_control/issues/707>`_)

  * [eus_qp/optmotiongen/euslisp/inverse-kinematics-wrapper.l] Fix load path
  * test-load-euslisp-files.l: run everything within euslisp code
  * [eus_qp] add test-load-euslisp-files.
  * [eus_qp/optmotiongen] add sqp-msc (multi solution candidates) feature. add manual and samples of sqp-msc.
  * [eus_qp/optmotiongen] fix variable name, _qp-retval.
  * [eus_qp/optmotiongen] set contact-ik-args name from argument.

* add two use case of optmotiongen (`#701 <https://github.com/jsk-ros-pkg/jsk_control/issues/701>`_)

  * [eus_qp/optmotiongen] update README, gif, and generate-gif.l to generate demo images"
  * [eus_qp/optmotiongen] set name of face and line contact from argument.
  * [eus_qp/optmotiongen] add demo-pr2-regrasp-object.l
  * [eus_qp/optmotiongen] update manual.pdf
  * [eus_qp/optmotiongen] add fetch demo using :inverse-kinematics-optmotiongen
  * [eus_qp/optmotiongen] add line and face demo with hrp2

* add optimization motion generation (`#700 <https://github.com/jsk-ros-pkg/jsk_control/issues/700>`_)

* Contributors: Kei Okada, Masaki Murooka, Naoya Yamaguchi, Riku Shigematsu, Satoshi Otsubo, Tatsuya Ishikawa, Weiqi Yang

0.1.15 (2018-05-16)
-------------------

0.1.14 (2018-01-15)
-------------------
* [eus_qp/euslisp/test-model-predictive-control.l] Add walking example by footstep list (`#682 <https://github.com/jsk-ros-pkg/jsk_control/issues/682>`_)
* Contributors: Shunichi Nozawa

0.1.13 (2017-04-18)
-------------------

0.1.12 (2017-02-22)
-------------------

0.1.11 (2017-02-09)
-------------------
* [eus_qp/README.md] Add readme for eus_qp and euslisp programs.
* [eus_qp/euslisp/cfr-cwc-calculation.l] Use obj env constraint in calc-dynamic-min-max-cog-pos
* [eus_qp/euslisp/cfr-cwc-calculation.l] Remove unused argument for calc-constraint-matrix-vector-for-obj-env-constraint
* Contributors: Shunichi Nozawa

0.1.10 (2016-12-15)
-------------------
* package.xml : remove unnecessary space within name tag
* eus_qp : add missing install directory
* [eus_qp/euslisp/contact-optimization.l,test-contact-wrench-opt.l] support rotational sliding in default-contact-constraint.
* [eus_qp/euslisp/contact-optimization.l,test-contact-wrench-opt.l] add rotational-sliding-contact constraint class and calc-constraint-param-list-for-rotational-sliding function.
* [eus_qp/euslisp/contact-optimization.l,test-contact-wrench-opt.l] use :fx, :-fx instead of :x, :-x.
* [test/test_cfr_cwc_calculation.l] Check version of jskeus for old deb installed environment (like hydro travis)
* [eus_qp/*/*cfr-cwc-calculation*, eus_qp/CMakeLists.txt] Add cfr and cwc calculation.
* [eus_qp/euslisp/model-predictive-control.l] Enable to add additional-inequality-matrix and additional-inequality-min-vector for MPC COG parameters
* [eus_qp/euslisp/contact-optimization.l] Add additional-inequality-matrix and additional-inequality-min-vector
* [eus_qp/euslisp/test-contact-wrench-opt.l,eus_qp/test/test_contact_wrench_opt.l] Add test for cop polygon and friction polycone constraint.
* [eus_qp/euslisp/contact-optimization] Add linearlized polycone friction constraint
* [eus_qp/euslisp/model-predictive-control.l] Just fix indent
* [eus_qp/euslisp, eus_qp/test] Add skip count for receding horizon proc count and update tests.
* [eus_qp/euslisp/contact-optimization.l] print message of too large equality error only when debug is true.
* [eus_qp/euslisp/model-predictive-control.l] Add argument to switch solve-qp-mode for qpoases
* [eus_qp/euslisp/contact-optimization.l] Enable to set solve-qp-mode. Default value is same as solve-qpoases-qp function
* [eus_qp/euslisp/test-contact-wrench-opt.l] Add test for use equality-error-weight
* [eus_qp/euslisp/contact-optimization.l] Add argument to support equality error.
* [eus_qp/euslisp/contact-optimization.l] add :gen-drawing-object method to 6d-min-max-contact-constraint
* Merge pull request `#574 <https://github.com/jsk-ros-pkg/jsk_control/issues/574>`_ from mmurooka/poly-cop
  [eus_qp/euslisp] add polygon-cop-contact-constraint
* [eus_qp/euslisp/contact-optimization.l, test-contact-wrench-opt.l, eus_qp/test/test_contact_wrench_opt.l] Add function to calculate wrench from given wrench. Currently calculation with contact constraints does not work.
* [euslisp/contact-optimization.l] add argument contact-face to default-contact-constraint.
* [euslisp/test-contact-wrench-opt.l] add test for polygon-cop-contact-constraint.
* [euslisp/contact-optimization.l] add class and function for polygon-cop-contact-constraint.
* Contributors: Kei Okada, Masaki Murooka, Shunichi Nozawa

0.1.9 (2016-03-23)
------------------
* Merge pull request #565 from mmurooka/6d-minmax-constraint
  [eus_qp/euslisp] add 6d-min-max-contact-constraint class and test
* [eus_qp/euslisp] pass debug option to qp solve function.
* [eus_qp/euslisp] add 6d-min-max-contact-constraint class and test for that.
* [eus_qp/euslisp/contact-optimization.l,test-contact-wrench-opt.l] Add contact constraint for hand gripper and add example.
* [eus_qp/euslisp/contact-optimization.l, test-contact-wrench-opt.l] Add mu-margin-ration and cop-margin-ratio. Add example for these parameters.
* [eus_qp/euslisp/contact-optimization.l] Fix updating of drawing object newcoords. If initialize, update coords.
* [eus_qp/euslisp/contact-optimization.l] Generate drawing object when drawing (https://github.com/jsk-ros-pkg/jsk_control/pull/558).
* [eus_qp/euslisp/contact-optimization.l] Enable to set jacobi from arg.
* [eus_qp/CMakeLists.txt, package.xml, src/qp_lib.cpp] Revert Eigen usage hack by garaemon, because we can build this program on travis without this hack (https://github.com/jsk-ros-pkg/jsk_control/commit/4937ac04d0c1beceb8c4c92eac258c00549943f9)
* [eus_qp/euslisp/contact-optimization.l] Just fix indent.
* [eus_qp/euslisp/contact-optimization.l] Add max fz if necessary
* [eus_qp/euslisp/model-predictive-control.l] Fix typos in MPC drift and output ports.
* [eus_qp/euslisp/model-predictive-control.l,test-model-predictive-control.l] Update mpc to fix calculation for drift and output matrices
* [eus_qp/test/test_model_predictive_control.test] Increase time-limit for MPC rostest
* [eus_qp/euslisp/contact-optimization.l] Define concatenate matrix function for old euslisp environment.
* [eus_qp/euslisp/test-model-predictive-control.l] Update for IK default argument for MPC examples.
* [eus_qp/euslisp/contact-optimization.l] Moved concatenate matrix functions to jskeus (https://github.com/euslisp/jskeus/commit/5b1cf86398c4688f41c6ec654c00059e5cbd7bca)
* [eus_qp/CMakeLists.txt] Add eus_qp MPC test for cmake rostest.
* Contributors: Shunichi Nozawa, Masaki Murooka

0.1.8 (2015-11-02)
------------------
* Merge pull request `#512 <https://github.com/jsk-ros-pkg/jsk_control/issues/512>`_ from k-okada/fix_error
  package.xml: add rostest to build_depend of eus_qp
* package.xml: add rostest to build_depend of eus_qp
* Contributors: Kei Okada

0.1.7 (2015-11-01)
------------------
* [3rdparty/eiquadprog.hpp] using std::abs instead of internal::abs
* CMakeLists.txt: using test as execute name may confuse system
* [eus_qp/euslisp/model-predictive-control.l] Support output variables in evaluation
* [eus_qp/euslisp/model-predictive-control.l] Return world input value (wrench)
* [eus_qp/euslisp/contact-optimization.l, model-predictive-control.l, test-model-predictive-control.l] Fix bug of mpc inequality-matrix contact coords and update samples
* [eus_qp/euslisp/*model-predictive-control.l, test/test_model_predictive_control.*, CMakeLists.txt] Add model predictive control from Nagasaka'2012 and add examples and tests.
* [eus_qp/euslisp/contact-optimization.l, eus_qp/euslisp/test-contact-wrench-opt.l] Add no-contact constraint and tests
* [../../eus_qp/euslisp/contact-optimization.l,test-eus-qpoases.l,eus-qpoases.l] Rename solve-qpoases => solve-qpoases-qp and remain solve-qpoases for backward compatibility with warning.
* Remove manifest.xml and Makefile and use catkin style filesystem
* Rename samplerobot demo function add infeasible sample. Add to rostest.
* Do not use immediate value for max demo function num
* Add test for force min violation
* Add inequality constraint violation mode if user set min-inequality-violation-weight.
* add sample for testing sliding contact constraint
* add translational sliding constraint to contact-optimization.l
* Add min-max constraint
* Use contact-constraint-vector-list
* Update test for test-contact-wrench-opt.l
* Add demo programe for all contact constraints
* Rename friction contact constraint
* Add constraint vector and use constraint-matrix slots variable
* Fix order of drawing
* Fix force color
* Add test for wrench contact application
* Add contact optimization application using euslisp qp calculation
* Contributors: Kei Okada, Ryohei Ueda, Shunichi Nozawa, Masaki Murooka

0.1.6 (2015-06-11)
------------------
* [eus_qp] Fix for indigo. Eigen3 on indigo may not provide Eigen::internal::sqrt
  Eigen::internal::abs, in order to provide them, we define these function in qp_lib.cpp
  before including qp stuff.

0.1.5 (2015-01-08)
------------------

0.1.4 (2014-10-21)
------------------
* add eigen to depend

0.1.3 (2014-10-10)
------------------

0.1.2 (2014-09-08)
------------------
* eigen is no longer ros package
* add catkin_package()
* Contributors: Kei Okada

0.1.1 (2014-09-04)
------------------
* use find_package(catkin COMPONENTS cmake_modules)
* add dependancies of euslisp and eigen
* bag fix load-library functions
* fix eiquadprog.l, plugin load from LD_LIBRARY_PATH
* add package.xml,
* add solve-eiquadprog-raw-with-error function, solve qp with error tolerance, usage=solve-eiquadprog :eiquadprog-function 'solve-eiquadprog-raw-with-error,
* bug fix of check_constraints function, args order change
* eq constraints check fix, but this is unbeliabable mistake, why it could be move?
* returns nil if eiquadprog is not solved
* fix args for qp_lib.cpp change
* add some comment, and constrants check result set in global value flag
* add constraints check functions
* remove unused comment
* fix debug mode stop the main functino
* rename state variable name from f0
* rename eq -> equality , non-eq -> inequality
* fix typo ;; min->max
* .l bug fix, eq constraints mean CEx + ce = 0
* fix test function, plus minus changed
* add Makefile
* add eus_qp dir, solve qp problem with euslisp, use eigenquadprog library
* Contributors: Shintaro Noda, Shunichi Nozawa
