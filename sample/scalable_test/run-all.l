#!/usr/bin/env roseus

(require "gen-ik-net.l")

(defun get-load-average
  nil
  (let* ((p (piped-fork
	     "uptime | sed -e \"s/^.\\\+load average: \\\([0-9\\\.]\\\+\\\), .\\\+$/\\\1/g\""))
	 ret)
    (setq ret (read-line p))
    (if (plusp (length ret))
	(setq ret (read-from-string ret)))
    (if (not (numberp ret))
	(progn (warning-message 1 "load-avaerage invalid string~A~%" ret)
	       (setq ret 0)))
    (close p)
    ret))

(let* ((lav (get-load-average)) (max 5.9) (skip (round (- max lav))))
  (dolist (param
	   (list (list 9 100 2 1 6 2 100)
		 (list 9 100 3 1 6 3 100)
		 (list 9 100 4 1 6 4 100)
		 (list 9 70 2 1 6 2 70)
		 (list 9 70 3 1 6 3 70)
		 (list 9 70 4 1 6 4 70)))
    (fileoutput-layer-string "ik_net.prototxt" (apply 'gen-redundancy-ik-net param))
    (fileoutput-layer-string "predict_ik_net.prototxt"
			     (apply 'gen-predict-ik-net param))
    (unix::system "./run_once.sh &")
    (if (plusp (+ 1 (decf skip))) (unix::sleep 3)
      (progn (unix::sleep (* 3 60))
	     (setq lav (get-load-average))
	     (setq skip (round (- max lav)))))
    ;;
    (warning-message 3 "load avarage ~A > ~A?~%" lav max)
    (while (> (setq lav (get-load-average)) max)
      (warning-message 3 "load avarage ~A > ~A~%" lav max)
      (unix::sleep 10)))
  )
