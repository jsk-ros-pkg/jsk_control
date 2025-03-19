#!/usr/bin/env roseus

(defun gen-plugin-obj
  (libname)
  (labels
      ((library_search
	(str &key
	     (depth 0)
	     colon-pos lib-path)
	(format t "  [~A] target=" depth)
	(cond
	 ((eq (length str) 0)
	  (format t "~% ~A not found~%" libname)
	  (exit -1))
	 ((and (setq colon-pos (or (position #\: str) (length str)))
	       (setq lib-path (subseq str 0 colon-pos))
	       (setq lib-path
		     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
			 (subseq lib-path 0 (- (length lib-path) 1))
		       lib-path))
	       (probe-file (setq lib-path
				 (print (format nil "~A/~A" lib-path libname)))))
	  (load-foreign lib-path))
	 (t
	  (library_search (subseq str (min (length str) (+ colon-pos 1)))
			  :depth (+ depth 1))))))
    (library_search (format nil "~A:~A/lib"
			    (unix:getenv "LD_LIBRARY_PATH")
			    (read-line (piped-fork "pwd"))))))


(defun random-sample-indices
  (size
   &key
   (min 0)
   (max (* size 10))
   (val)
   size1 size2
   )
  (cond
   ((eq size 0) nil)
   ((eq max min)
    ;; (format t "impossible bacause size=0 vs ~A~%"
    ;; size)
    nil)
   (t
    (setq val (+ min (random (- max min))))
    (setq size1
	  (round
	   (* (- size 1)
	      (/ (- val min) (- max min 1.0)))))
    (append
     (random-sample-indices
      size1 :min min :max val)
     (list val)
     (random-sample-indices
      (- (- size 1) size1) :min (+ val 1) :max max))
    )))

(defun random-vector-swap
  (vec &key (cnt 1))
  (dotimes (i cnt)
    (let* ((id1 (random (length vec)))
	   (id2 (random (length vec)))
	   (buf (aref vec id1)))
      (setf (aref vec id1) (aref vec id2))
      (setf (aref vec id2) buf)))
  vec)
