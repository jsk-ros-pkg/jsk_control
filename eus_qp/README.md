eus_qp
=====================

## Features
  * QP and optimization library and Euslisp bindings
  * Euslisp libraries using optimization

## QP and optimization library and Euslisp bindings
  * [eus + eiquadprog] (https://github.com/jsk-ros-pkg/jsk_control/blob/master/eus_qp/euslisp/eiquadprog.l)
  * [eus + qpoases (eus_qpoases package)] (https://github.com/jsk-ros-pkg/jsk_control/blob/master/eus_qpoases)

## Euslisp libraries using optimization
  * [Contact optimization] (https://github.com/jsk-ros-pkg/jsk_control/blob/master/eus_qp/euslisp/contact-optimization.l)  
      * Calculation function for wrench and torque distribution.
      * Define classes to represent contact
  * [Model predictive control] (https://github.com/jsk-ros-pkg/jsk_control/blob/master/eus_qp/euslisp/model-predictive-control.l)
      * MPC base class
      * MPC for COM motion generation
  * [CFR and CWC calculation] (https://github.com/jsk-ros-pkg/jsk_control/blob/master/eus_qp/euslisp/cfr-cwc-calculation.l)
      * Calculate functions for CFR (Static or Dynamic COM Feasible Region, ZMP, ...) using LP etc
      * Utilities using optimization
  * Tests
      * `euslisp/test-**.l` are samples and tests for above codes.
      * These test codes are tested as rostest in `test/test_*.l` and `test/test_*.test`.
