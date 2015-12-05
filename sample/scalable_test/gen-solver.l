#!/usr/bin/env roseus

(defun gen-solver
  (&key
   (net "\"ik_net.prototxt\"")
   (solver_type "NESTEROV")
   (base_lr 0.1)
   (stop_lr (* base_lr 0.1))
   (max_iter 100000)
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

(defun parse-solver
  (&key
   (path "ik_solver.best.prototxt"))
  (let* ((p (open path :direction :input))
	 buf tmp compos tag value)
    (while (setq tmp (read-line p nil))
      (setq compos (position #\: tmp))
      (cond
       ((minusp compos)
	(warning-message 3 "tag: value format denied~%"))
       (t
	(setq tag (subseq tmp 0 compos))
	(setq value (subseq tmp (+ compos 1)))
	(if (plusp (length tag)) (setq tag (read-from-string tag)))
	(if (plusp (length value)) (setq value (read-from-string value)))
	(push (cons tag value) buf))))
    (close p)
    buf))

(defun update-solver
  (&rest
   args
   &key
   (input "ik_solver.best.prototxt")
   (output-surfix (car (unix::gettimeofday)))
   (output (format nil "~A.~A" input output-surfix))
   (data (parse-solver :path input))
   (base_lr (cdr (assoc 'base_lr data)))
   (max_iter (cdr (assoc 'max_iter data)))
   (stepsize (cdr (assoc 'stepsize data)))
   (gamma (cdr (assoc 'gamma data)))
   (start_iter max_iter)
   &allow-other-keys
   )
  (let* ((p (open output :direction :output)))
  (dolist (d data)
    (format p "~A: ~A~%" (car d)
	    (let* ((val
		    (cond
		     ((assoc (car d) args) (cdr (assoc (car d) args)))
		     ((eq (car d) 'max_iter)
		      (+ max_iter start_iter))
		     ((eq (car d) 'base_lr)
		      (/ base_lr (expt gamma (/ (* max_iter 1.0) stepsize))))
		     (t (cdr d)))))
	      (cond
	       ((stringp val) (format nil "\"~A\"" val))
	       ((symbolp val) (send val :pname))
	       (t val)))))
  (close p)
  output
  ))
