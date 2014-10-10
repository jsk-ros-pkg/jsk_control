^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package eus_qp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
