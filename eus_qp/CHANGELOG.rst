^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_qp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
