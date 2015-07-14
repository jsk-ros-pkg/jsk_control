#!/usr/bin/env roseus
(comp:compile-file-if-src-newer
  (format nil "~A/euslisp/sample_astar_func.l"
          (ros::rospack-find "jsk_footstep_planner")))
(let ((old-module (find "sample_astar_func" *loaded-modules*
                        :key #'lisp::load-module-file-name :test #'equal)))
  (if old-module (sys::unbinload old-module)))
(load "sample_astar_func")
(run)