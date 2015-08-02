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
