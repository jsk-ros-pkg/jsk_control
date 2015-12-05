#!/usr/bin/env roseus

(defun gen-solver
  (&key
   (net "\"ik_net.prototxt\"")
   (solver_type "NESTEROV")
   (base_lr 0.1)
   (stop_lr (* base_lr 0.1))
   (max_iter 10000)
   (stepsize 1000)
   (display stepsize)
   (snapshot max_iter)
   (lr_policy "\"step\"")
   (gamma (expt (/ stop_lr base_lr) (/ (* stepsize 1.0) max_iter)))
   (snapshot_prefix "\"snapshot\"")
   (solver_mode "CPU")
   (momentum 0.9)
   (path "ik_solver.prototxt"))
  (let* ((p (open path :direction :output)))
    (mapcar
     #'(lambda (sym)
	 (format p "~A: ~A~%" sym (eval sym)))
     '(net solver_type base_lr lr_policy gamma stepsize max_iter momentum display snapshot snapshot_prefix solver_mode))
    (close p)
    ))
