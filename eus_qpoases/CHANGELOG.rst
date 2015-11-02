^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_qpoases
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* [eus_qpoases] use ExternalProject instead of mk
* [eus_qpoases/euslisp/eus-qpoases.l] Check equality usage and use state-dim and inequality-dim.
* [../../eus_qp/euslisp/contact-optimization.l,test-eus-qpoases.l,eus-qpoases.l] Rename solve-qpoases => solve-qpoases-qp and remain solve-qpoases for backward compatibility with warning.
* [../src/eus_qpoases.cpp,../test/eus-qpoases.test.l,test-eus-qpoases.l,eus-qpoases.l] Add LP solver using qpOASES and add samples and tests.
* [../src/eus_qpoases.cpp,../test/eus-qpoases.test.l,test-eus-qpoases.l,eus-qpoases.l] Add LP solver using qpOASES and add samples and tests.
* [eus_qpoases/euslisp/eus-qpoases.l] Add argument matrix size check
* Remove manifest.xml and Makefile and use catkin style filesystem
* Add comments for qp functions
* Add demo-eus-qpOASES4 for force infeasible problem.
* Print debug message by default. If you want to disable it, :debug nil
* Add debug print check
* Added install rules for qpoases headers and libs to catkin cmake-file.
* require rostest in cmake files
* add rostest declaration
* intial commit of test dir and test files, check euslisp/test-eus-qpoases.l all functions, and if there are at least one failure, then test would be failed
* test functions return test results
* [eus_qpoases] add dependencies to linqpOASES
* Contributors: Yuki Furuta, Georg Bartels, Ryohei Ueda, Shunichi Nozawa, Shintaro Noda

0.1.6 (2015-06-11)
------------------
* [eus_qpoases] Ignore bin directory
* [eus_qpoases] Support ccache

0.1.5 (2015-01-08)
------------------
* Get qpOASES status from argument
* Update patch for qpoases shared object (https://github.com/jsk-ros-pkg/jsk_control/issues/180)
* Contributors: Shunichi Nozawa

0.1.4 (2014-10-21)
------------------
* trust server ssl

0.1.3 (2014-10-10)
------------------

0.1.2 (2014-09-08)
------------------
* add run_depend to euslisp
* add subversion to build_depend
* Contributors: Kei Okada

0.1.1 (2014-09-04)
------------------
* Add euslisp package for qpOASES, which is a qp solver
* Contributors: Shunichi Nozawa
