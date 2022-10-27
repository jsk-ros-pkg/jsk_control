^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_qpoases
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.16 (2022-10-27)
-------------------
* https://projects.coin-or.org have mved to github (`#776 <https://github.com/jsk-ros-pkg/jsk_control/issues/776>`_)

* add optimization motion generation (`#700 <https://github.com/jsk-ros-pkg/jsk_control/issues/700>`_)

  * [eus_qpoases/src/eus_qpoases.cpp] change nWSR for large size QP problem: 1000 -> 10000

* Contributors: Kei Okada, Masaki Murooka

0.1.15 (2018-05-16)
-------------------

0.1.14 (2018-01-15)
-------------------

0.1.13 (2017-04-18)
-------------------

0.1.12 (2017-02-22)
-------------------
* mv .gitignore to .placeholder
* Contributors: Kei Okada

0.1.11 (2017-02-09)
-------------------
* Set include path of qpoases in eus_qpoases
* Contributors: Kei Okada, Kentaro Wada

0.1.10 (2016-12-15)
-------------------
* eus_qpoases : add missing install directory
* [euslisp/test-eus-qpoases.l,test/eus-qpoases.test.l] Add test for sqp/slp
* [eus_qpoases.cpp, eus-qpoases.l] Add qp with hotstart and sqp with hotstart
* [eus_qpoases/src/eus_qpoases.cpp] Use common function for qp and lp.
* Contributors: Kei Okada, Shunichi Nozawa

0.1.9 (2016-03-23)
------------------
* [eus_qpoases/euslisp/eus-qpoases.l] Use concatenate matrix functions in irteus and fix checking of ineq and eq existence
* Contributors: Shunichi Nozawa

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
