^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_nlopt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.17 (2023-05-28)
-------------------
* use message(WARNING for 'NLopt missing'
* Contributors: Kei Okada

0.1.16 (2022-10-27)
-------------------
* use System nlopt library for noetic, skip compile if nlopt is missing (`#773 <https://github.com/jsk-ros-pkg/jsk_control/issues/773>`_)
* Contributors: Kei Okada

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
* add install rules
* Contributors: Kei Okada

0.1.10 (2016-12-15)
-------------------

0.1.9 (2016-03-23)
------------------

0.1.8 (2015-11-02)
------------------

0.1.7 (2015-11-01)
------------------
* use ftol xtol values in coresolver initialization
* Remove manifest.xml and Makefile and use catkin style filesystem
* Contributors: Ryohei Ueda, Shintaro Noda

0.1.6 (2015-06-11)
------------------

0.1.5 (2015-01-08)
------------------
* remove engin from build_depend
* remove find_package(Eigen)
* Contributors: Kei Okada

0.1.4 (2014-10-21)
------------------
* add eigen

0.1.3 (2014-10-10)
------------------
* Use deb files to install jsk programs
* Fix path to find nlopt headers and libraries

0.1.2 (2014-09-08)
------------------

0.1.1 (2014-09-04)
------------------
* catkinize, add package.xml and catkin.cmake, use ROS_BUILD flag, and fix lib loader to use LD_LIBRARY_PATH
* rename arg names, x -> state-vector, f -> evaluation function, g -> equality function, h -> inequality function
* remove test dir
* rename m -> *nlopt-plugin*
* add solve-nlopt-ik-thread function
* add callback function
* fix env value,
* add axis-matrix arg,
* if response is not best, revert robot angle-vector
* revert Makefile
* chmode a+x node & maanger
* delete some files
* add launch file, solve ik separatery and return the best result
* bug fix, nlopt-ik-manager
* add nlopt-ik-manager.l, combine the ik result and publish the best one
* some minor changes hogehoge
* add nlopt-ik-node.l, solve ik with s equation
* move atlas-ik* files to ik dir
* add nlopt-ik-overwrite.l, causion, this program overwrite the method of :inverse-kinematics in cascaded-link
* list check add
* enable to use eus-ik with algoright id > 18
* base min max value add
* add debug-view=:success-draw mode, draw only when the const value is updated
* fix maxeval value, enough large
* add ik-test function for check
* bug fix: root-link has no joint
* bug fix: target-centroid-pos can be used
* bug fix: ik-param send super nlopt-object
* add centroid control, bug has some bug
* bug fix: requrie path change from . to ..
* initial commit nlopt-ik.l
* move test files into test dir
* nlopt-object: able to set max-time and max-eval
* add max-eval, max-time variable
* bug fix: nlopt-fdh return interger
* :simple-jacobian function add
* object wrapper for nlotp add
* euslisp global value add for local non-derivetive functinos
* add local non-derivetive functions, with no test
* remove duplicate simple-jacobian function
* update simple-jacobina, able to use for matrix jacobian
* add Makefile, just rosmake command build all files
* bug fix: cmakelist.txt missing links
* initial commit eus_nlopt, ik ssample add
* Contributors: Shintaro Noda
