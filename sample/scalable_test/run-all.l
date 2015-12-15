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

(let* ((lav (get-load-average)) (max 49.9) (skip (round (- max lav)))
       (s 16) (k 12))
  (dolist (m '(70 120))
    (dolist (i '(3 2 1))
      (dolist (param
               (list ;;(list s m 1 i k 2 m)
                     ;;(list s m 2 i k 2 m)
                     (list s m 1 i k 3 m)
                     (list s m 2 i k 3 m)
                     (list s m 1 i k 4 m)
                     (list s m 3 i k 4 m))
               )
        (fileoutput-layer-string "ik_net.prototxt" (apply 'gen-redundancy-ik-net param))
        (fileoutput-layer-string "predict_ik_net.prototxt"
                                 (apply 'gen-predict-ik-net param))
        (unix::system "./run_once.sh &")
        (if (plusp (+ 1 (decf skip))) (unix::sleep 10)
          (progn (unix::sleep (* 3 60))
                 (setq lav (get-load-average))
                 (setq skip (round (- max lav)))))
        (unix::sleep 3)
        ;;
        (warning-message 3 "load avarage ~A > ~A?~%" lav max)
        (while (> (setq lav (get-load-average)) max)
          (warning-message 3 "load avarage ~A > ~A~%" lav max)
          (unix::sleep 10)))
      )))
