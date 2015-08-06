#!/usr/bin/env roseus

(require "../learn.l")

(load-ik-learning-data)

(let* ((net-size (or (unix::getenv "NET_SIZE") "5x400")))
  (caffe::gen-test-net
   :netproto (format nil "ik_net_~A_predict.prototxt" net-size)
   :caffemodel (format nil "ik_net_~A_iter_3000000.caffemodel" net-size)))

(let* ((size (/ (length *ef-coords-map*) 6))
       (dif-max (float-vector 0 0 0 0 0 0))
       (dif-sum (float-vector 0 0 0 0 0 0))
       ret dif tm (avtm 0)
       )
  (dotimes (i size)
    (setq ret (ik-test i 'caffe::calc-forward-double nil))
    (setq dif (cdr (assoc :diff ret)))
    (setq tm (cdr (assoc :time ret)))
    (dotimes (j (length dif))
      (setf (aref dif-max j)
	    (max (abs (aref dif j)) (aref dif-max j)))
      (setf (aref dif-sum j)
	    (+ (/ (abs (aref dif j)) size)
	       (aref dif-sum j))))
    (setq avtm (+ avtm (/ tm size)))
    (format t "tm: ~A, maxd: ~A, sumd: ~A~%" avtm dif-max dif-sum)
    ;;
    ;;
    )
  (format t "tm: ~A, maxd: ~A, sumd: ~A~%" avtm dif-max dif-sum)
  (list avtm dif-max dif-sum)
  )
