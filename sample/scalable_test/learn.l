#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://eus_caffe/euslisp/eus-log.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "models/arrow-object.l")
(require "package://eus_caffe/sample/scalable_test/gen-solver.l")

(defvar *robot* (hrp2jsknt))
(defvar *joint-list1* (append (butlast (send *robot* :rarm :joint-list))
			      (send *robot* :torso :joint-list)))
(defvar *joint-list2* (append (butlast (send *robot* :rarm :joint-list))
			      (butlast (send *robot* :larm :joint-list))
			      (send *robot* :torso :joint-list)))
(defvar *joint-list3* (append (butlast (send *robot* :rarm :joint-list))
			      (butlast (send *robot* :larm :joint-list))
			      (butlast (send *robot* :rleg :joint-list))
			      (butlast (send *robot* :lleg :joint-list))
			      (send *robot* :torso :joint-list)))

(defvar *ef-coords* (list (send *robot* :rarm :end-coords)))
(defvar *arrow* (mapcar '(lambda (ec) (arrow)) *ef-coords*))

(objects (flatten (list (car (send *robot* :links))
			(reverse (send-all *joint-list3* :child-link))
			*arrow*)))

(defvar *sample-points* ;;96)
  (expt (max
  	 (min (round (expt 1e+8 (/ 1.0 (length *joint-list1*))))
  	      4) 2) (length *joint-list1*)))

(defvar *angle-vector-map*)
(defvar *ef-coords-map*)

(defun shuffle-random-vector-map
  (&key
   (sample-points *sample-points*)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (av-map *angle-vector-map*)
   (cc-map *ef-coords-map*)
   )
  (labels
   ((swap (a i j)
	  (let* ((c (aref a i)))
	    (setf (aref a i) (aref a j))
	    (setf (aref a j) c))))
   (dotimes (i sample-points)
     (let* ((r (random sample-points)))
       (dotimes (j (length joint-list))
	 (swap av-map
	       (+ j (* (length joint-list) i))
	       (+ j (* (length joint-list) r))))
       (dotimes (j (length ef-coords))
	 (dotimes (k 6)
	   (swap cc-map
		 (+ k (* 6 (length ef-coords) i))
		 (+ k (* 6 (length ef-coords) r)))))))))

(defun gen-ik-learning-data
  (&key (sample-points *sample-points*)
	(joint-list *joint-list1*)
	(ef-coords *ef-coords*)
	(sep (round (expt sample-points (/ 1.0 (length joint-list)))))
	(root-link (car (send *robot* :links)))
	(i 0)
	(draw? nil)
	(surfix ""))
  (setq *sample-points* sample-points)
  (if (not (eq (length *angle-vector-map*)
	       (* (length joint-list) sample-points)))
      (setq *angle-vector-map*
	    (instantiate float-vector
			 (* (length joint-list) sample-points))))
  (if (not (eq (length *ef-coords-map*)
	       (* 6 (length ef-coords) sample-points)))
      (setq *ef-coords-map*
	    (instantiate float-vector
			 (* 6 (length ef-coords) sample-points))))
  (labels ((itter
	    (jlist)
	    (cond
	     ((null jlist)
	      (let* (pos rot)
		(send-all (send-all joint-list :child-link) :worldcoords)
		(send-all ef-coords :worldcoords)
		(cond
		 (draw?
		  (send *viewer* :draw-objects :flush nil)
		  (send-all ef-coords :draw-on :flush nil
			    :color (float-vector 1 0 0) :size 100)
		  (send *viewer* :viewsurface :flush)))
		(dotimes (j (length joint-list))
		  (setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
			(deg2rad (send (nth j joint-list) :joint-angle))))
		(dotimes (j (length ef-coords))
		  (setq pos (send (send root-link :worldcoords)
				  :difference-position
				  (send (nth j ef-coords) :worldcoords)))
		  (setq rot (send (send root-link :worldcoords)
				  :difference-rotation
				  (send (nth j ef-coords) :worldcoords)))
		  (dotimes (k 3)
		    (setf (aref *ef-coords-map*
				(+ k (* j 6) (* i (length ef-coords) 6)))
			  (* 1e-3 (aref pos k))))
		  (dotimes (k 3)
		    (setf (aref *ef-coords-map*
				(+ 3 k (* j 6) (* i (length ef-coords) 6)))
			  (aref rot k)))
		  )
		(setq i (+ i 1))))
	     (t
	      (dotimes (j sep)
		(send (car jlist) :joint-angle
		      (+ (send (car jlist) :min-angle)
			 (* (/ j (- sep 1.0))
			    (- (send (car jlist) :max-angle)
			       (send (car jlist) :min-angle)))))
		(itter (cdr jlist))))
	     )))
	  (itter joint-list)
	  (shuffle-random-vector-map)
	  (dump-ik-learning-data :surfix surfix)
	  ))

(defun gen-random-ik-learning-data
  (&key (sample-points *sample-points*)
	(joint-list *joint-list1*)
	(ef-coords *ef-coords*)
	(sep (round (expt sample-points (/ 1.0 (length joint-list)))))
	(root-link (car (send *robot* :links)))
	(i 0)
	(draw? nil)
	(surfix (format nil ".~A" (unix-command-line "basename `pwd`"))))
  (setq *sample-points* sample-points)
  (if (not (eq (length *angle-vector-map*)
	       (* (length joint-list) sample-points)))
      (setq *angle-vector-map*
	    (instantiate float-vector
			 (* (length joint-list) sample-points))))
  (if (not (eq (length *ef-coords-map*)
	       (* 6 (length ef-coords) sample-points)))
      (setq *ef-coords-map*
	    (instantiate float-vector
			 (* 6 (length ef-coords) sample-points))))
  ;;
  (let* (pos rot)
    (dotimes (i sample-points)
      (dolist (j joint-list)
	(send j :joint-angle
	      (-
	       (+ (send j :min-angle)
		  (* (random 1.0)
		     (- (send j :max-angle) (send j :min-angle))))
	       (send j :joint-angle))
	      :relative t)
	)
      (send-all (send-all joint-list :child-link) :worldcoords)
      (send-all ef-coords :worldcoords)
      (dotimes (j (length joint-list))
	(setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
	      (deg2rad (send (nth j joint-list) :joint-angle))))
      (dotimes (j (length ef-coords))
	(setq pos (send (send root-link :worldcoords)
			:difference-position
			(send (nth j ef-coords) :worldcoords)))
	(setq rot (send (send root-link :worldcoords)
			:difference-rotation
			(send (nth j ef-coords) :worldcoords)))
	(dotimes (k 3)
	  (setf (aref *ef-coords-map*
		      (+ k (* j 6) (* i (length ef-coords) 6)))
		(* 1e-3 (aref pos k))))
	(dotimes (k 3)
	  (setf (aref *ef-coords-map*
		      (+ 3 k (* j 6) (* i (length ef-coords) 6)))
		(aref rot k)))
	))
    (dump-ik-learning-data :surfix surfix)
    ))

(defun load-ik-learning-data
  (&key (root "teacher") (surfix ""))
  (setq *angle-vector-map* (cpplog::read-vector (format nil "~A/angle-vector-map.dat~A" root surfix)))
  (setq *ef-coords-map* (cpplog::read-vector (format nil "~A/ef-coords-map.dat~A" root surfix)))
  (length *angle-vector-map*))

(defun dump-ik-learning-data
  (&key (root "teacher") (surfix ""))
  (unix::system (format nil "mkdir -p ~A" root))
  (cpplog::write-vector (format nil "~A/angle-vector-map.dat~A" root surfix)
			*angle-vector-map*)
  (cpplog::write-vector (format nil "~A/ef-coords-map.dat~A" root surfix)
			*ef-coords-map*))

(defun ik-learn
  (&key (solver "ik_solver.prototxt")
	(solverstate "")
	&allow-other-keys)
  (cond
   ((or (zerop (length *ef-coords-map*))
	(zerop (length *angle-vector-map*)))
    (load-ik-learning-data)
    (setq *sample-points* ;;96)
	  (/ (length *angle-vector-map*)
	     (length *joint-list1*)))
    ))
  ;; (caffe::create-solver :solver solver :solverstate "")
  ;; (caffe::reset-memory-layer :name "input"
  ;; 			     :size (length *angle-vector-map*)
  ;; 			     :data *angle-vector-map*)
  ;; (caffe::reset-memory-layer :name "target"
  ;; 			     :size (length *ef-coords-map*)
  ;; 			     :data *ef-coords-map*)
  (caffe::learn
   :solver solver
   :solverstate solverstate
   :size *sample-points*
   :ddata *ef-coords-map*
   :idata *angle-vector-map*
   ))

(defun unix-command-line
  (cmd)
  (let* ((p (piped-fork cmd))
	 (ret (read-line p nil)))
    (close p)
    ret))

(defun gen-snapshot-prefix
  (solver)
  (unix-command-line
   (concatenate string
		(format nil "cat ~A" solver)
		" | "
		"grep \"snapshot_prefix\""
		" | "
		"sed -e \"s/^snapshot_prefix: \\\"\\\(.\\\+\\\)\\\"$/\\\1/g\""
		)))

(defun link-best-settings
  (solver)
  (let* ((best-path solver)
	 (best-snap (gen-snapshot-prefix best-path))
	 (caffe-snap
	  (unix-command-line
	   (format nil "ls ~A*caffemodel -t | head -n 1" best-snap)))
	 (solver-snap
	  (unix-command-line
	   (format nil "ls ~A*solverstate -t | head -n 1" best-snap)))
	 (solver-lnk
	  (format nil "ln -snf ~A ik_solver.best.prototxt" best-path))
	 (caffe-lnk
	  (format nil "ln -snf ~A ik.best.caffemodel" caffe-snap))
	 (solverstate-lnk
	  (format nil "ln -snf ~A ik.best.solverstate" solver-snap)))
    (warning-message 3 (format nil "~A~%" solver-lnk))
    (if (string-equal solver "ik_solver.best.prototxt")
	(print 'skip)
      (unix::system solver-lnk))
    (warning-message 3 (format nil "~A~%" caffe-lnk))
    (unix::system caffe-lnk)
    (warning-message 3 (format nil "~A~%" solverstate-lnk))
    (unix::system solverstate-lnk)
    ))

(defun ik-learn-with-all-solver
  (&rest args)
  (let* ((ret
	  (sort
	   (mapcar
	    #'(lambda (solver)
		(cons
		 solver
		 (apply 'ik-learn :solver solver args)))
	    (remove-if-not
	     #'(lambda (str) (and (substringp "ik_solver" str)
				  (substringp "prototxt" str)))
	     (directory)))
	   #'(lambda (a b) (< (cdr a) (cdr b)))))
	 )
    (link-best-settings (car (car ret)))
    ret))

(defun ik-learn-best
  (&rest args)
  (let* (config state)
    ;; detect solver config
    (setq config
	  (or (unix-command-line "ls ik_solver.best.prototxt")
	      (unix-command-line "ls -t ik_solver*prototxt")))
    ;;
    ;; detect solver state
    (setq state
	  (or (unix-command-line "ls ik.best.solverstate")
	      (unix-command-line "ls -t ik*solverstate")
	      ""))
    ;;
    (warning-message 3 "config: ~A, state: ~A~%" config state)
    ;;
    (cond
     ((not (and config state))
      (warning-message 1 "empty setting files~%")
      (throw :settings-not-found nil)))
    (warning-message 3 "regenrate random traning DB~%")
    (gen-random-ik-learning-data)
    (warning-message 3 "update solver~%")
    (let* ((p (apply 'update-solver :input "ik_solver.best.prototxt" args)))
      (unix::system (format nil "ln -snf ~A ik_solver.best.prototxt" p)))
    (warning-message 3 "start learning ... ~%")
    (apply 'ik-learn :solver config :solverstate state args)
    ;; (ik-learn :solver solver :solverstate state)
    (warning-message 3 "done~%")
    (link-best-settings "ik_solver.best.prototxt")
    (warning-message 3 "evaluate network~%")
    (ik-random-test-min-average)
    ))

(defun ik-learn-best-loop
  (&rest args &key (lp 10) &allow-other-keys)
  (let* ((ret))
    (dotimes (i lp)
      (push (apply 'ik-learn-best args) ret)
      (if (not ret) (return-from nil nil))
      )
    (setq *ret* ret)
    ret))

(defun check-test-data
  (&optional
   (i (random *sample-points*)) ;;0)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (input (instantiate float-vector (* (length ef-coords) 6)))
   (output (instantiate float-vector (length joint-list)))
   c c-buf
   )
  (dotimes (j (length ef-coords))
    (dotimes (k 6)
      (setf (aref input (+ k (* j 6)))
	    (aref *ef-coords-map* (+ k (* i (length input)) (* j 6)))))
    (setq c (make-coords
	     :pos (scale 1e+3 (subseq input (+ 0 (* j 6)) (+ 3 (* j 6))))
	     :rot (matrix-exponent (subseq input (+ 3 (* j 6)) (+ 6 (* j 6))))))
    (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform c :local))
    (push c c-buf)
    )
  (setq c-buf (reverse c-buf))
  (dotimes (j (length joint-list))
    (setf (aref output j)
	  (aref *angle-vector-map* (+ (* i (length joint-list)) j))))
  (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a)))
       joint-list output)
  (send-all (send-all joint-list :child-link) :worldcoords)
  (send-all ef-coords :worldcoords)
  (mapcar
   '(lambda (ar c)
      (send ar :newcoords (send c :copy-worldcoords)))
   *arrow* c-buf)
  (send *viewer* :draw-objects :flush nil)
  (send *viewer* :viewsurface :flush)
  (print (apply 'concatenate (cons float-vector (flatten (mapcar '(lambda (ec c) (list (send c :difference-position ec) (send c :difference-rotation ec))) ef-coords c-buf)))))
  )

;; (defun ik-test
;;   (&optional
;;    (i 0)
;;    (calc-forward 'caffe::calc-forward-double)
;;    (draw? t)
;;    ;;
;;    (input (instantiate float-vector 6))
;;    (output (instantiate float-vector 6))
;;    c tm
;;    )
;;   (dotimes (j 6) (setf (aref input j) (aref *ef-coords-map* (+ (* i 6) j))))
;;   (setq c (make-coords :pos (scale 1e+3 (subseq input 0 3))
;; 		       :rot (matrix-exponent (subseq input 3 6))))
;;   (setq c (send (send (car (send *robot* :links)) :copy-worldcoords)
;; 		:transform c :local))
;;   (setq
;;    tm
;;    (bench2
;;     (bench
;;      (apply calc-forward
;; 	    (list :isize (length input) :input input
;; 		  :osize (length output) :output output)))))
;;   (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a)))
;;        *rarm-joints* output)
;;   (cond
;;    (draw?
;;     ;; (send *viewer* :draw-objects :flush nil)
;;     ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
;;     (send *arrow* :newcoords (send c :copy-worldcoords))
;;     (send *viewer* :draw-objects :flush nil)
;;     (send *viewer* :viewsurface :flush)))
;;   (list
;;    (cons :time tm)
;;    (cons :diff
;; 	 (print (concatenate
;; 		 float-vector
;; 		 (send c :difference-position (send *robot* :rarm :end-coords))
;; 		 (send c :difference-rotation (send *robot* :rarm :end-coords))))))
;;   )

(defun smart-initialize-net
  nil
  (caffe::gen-test-net :netproto "predict_ik_net.prototxt"
		       :caffemodel
		       (read-line
			(piped-fork
			 "ls -t | grep -e \"caffemodel$\" | head -n 1")))
  ;; (caffe::create-solver :solver "ik_solver_batch1.prototxt"
  ;; 			:solverstate
  ;; 			(read-line
  ;; 			 (piped-fork
  ;; 			  "ls -t | grep -e \"solverstate$\" | head -n 1")))
  ;; (caffe::reset-memory-layer :name "input"
  ;; 			     :size (length av)
  ;; 			     :data (copy-seq av))
  ;; (caffe::reset-memory-layer :name "target"
  ;; 			     :size (length input)
  ;; 			     :data (copy-seq input))
  )

(defun ik-random-test
  (&key (calc-forward 'caffe::calc-forward-double) ;;'caffe::memory-calc-forward)
	(draw? t)
	tm
	(joint-list *joint-list1*)
	(ef-coords *ef-coords*)
	(random-angle
	 (mapcar '(lambda (j) (send j :joint-angle
				    (+ (send j :min-angle)
				       (* (random 1.0)
					  (- (send j :max-angle)
					     (send j :min-angle))))))
		 joint-list))
	(update-coords
	 (progn (send-all (send-all joint-list :child-link) :worldcoords)
		(send-all ef-coords :worldcoords)))
	(c (send-all ef-coords :copy-worldcoords))
	(b (send (car (send *robot* :links)) :copy-worldcoords))
	(av (instantiate float-vector (length joint-list)))
	(input
	 (apply 'concatenate
		(cons float-vector
		      (flatten
		       (mapcar #'(lambda (c)
				   (list
				    (scale 1e-3 (send b :difference-position c))
				    (send b :difference-rotation c))) c)))))
	(redundancy (float-vector 0.5))
	(initialize? (smart-initialize-net))
	&allow-other-keys
	)
  (setq input (concatenate float-vector input redundancy))
  (setq
   tm
   (bench2
    (bench
     (apply calc-forward
	    (list :isize (length input) :input input
		  :osize (length av) :output av))
     )))
  (map cons #'(lambda (j a) (send j :joint-angle (rad2deg a))) joint-list av)
  (cond
   (draw?
    ;; (send *viewer* :draw-objects :flush nil)
    ;; (send c :draw-on :flush nil :color (float-vector 1 0 0) :width 100)
    (mapcar
     '(lambda (ar c)
	(send ar :newcoords (send c :copy-worldcoords)))
     *arrow* c)
    (send *viewer* :draw-objects :flush nil)
    (send *viewer* :viewsurface :flush)))
  (list
   (cons :time tm)
   (cons :task-diff
	 (apply 'concatenate (cons float-vector (flatten (mapcar '(lambda (ec c) (list (scale 1e-3 (send c :difference-position ec)) (send c :difference-rotation ec))) ef-coords c))))
	 )
   (cons :state-diff
	 (map float-vector '(lambda (a b) (- (deg2rad a) b))
	      random-angle av))
   (cons :state-diff-norm
	 (sqrt
	  (/ (norm2 (map float-vector '(lambda (a b) (- (deg2rad a) b))
			 random-angle av)
		    )
	     (length av))))
   ))

(defun ik-random-test-min
  (&rest
   args
   &key
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (random-angle
    (mapcar '(lambda (j) (send j :joint-angle
			       (+ (send j :min-angle)
				  (* (random 1.0)
				     (- (send j :max-angle)
					(send j :min-angle))))))
	    joint-list))
   (update-coords
    (progn (send-all (send-all joint-list :child-link) :worldcoords)
	   (send-all ef-coords :worldcoords)))
   (c (send-all ef-coords :copy-worldcoords))
   (b (send (car (send *robot* :links)) :copy-worldcoords))
   (av (instantiate float-vector (length joint-list)))
   (input
    (apply 'concatenate
	   (cons float-vector
		 (flatten
		  (mapcar #'(lambda (c)
			      (list
			       (scale 1e-3 (send b :difference-position c))
			       (send b :difference-rotation c))) c)))))
   (redundancy-step 0.1)
   (redundancy (float-vector 0.0))
   (initialize? (smart-initialize-net))
   buf
   )
  (dotimes (i (length redundancy))
    (dotimes (j (round (/ 1.0 redundancy-step)))
      (setf (aref redundancy i) (* j redundancy-step))
      (push (apply 'ik-random-test :random-angle random-angle :update-coords update-coords :c c :b b :av av :input input :redundancy redundancy :initialize? nil args) buf)))
  (setq *ret* buf)
  (car (sort buf #'(lambda (a b) (< (cdr (assoc :state-diff-norm a))
				    (cdr (assoc :state-diff-norm b))))))
  )

(defun ik-random-test-min-average
  (&optional (lp 100))
  (let* ((ret))
    (smart-initialize-net)
    (dotimes (i lp)
      (push (ik-random-test-min :initialize? nil) ret))
    (setq *ret* ret)
    (/ (apply '+ (mapcar '(lambda (d) (cdr (assoc :state-diff-norm d))) ret))
       (length ret))))

#|

(defun random-index-vector
  (size
   &key
   (ret (instantiate integer-vector size))
   (cnt (* 10 size)))
  (dotimes (i size) (setf (aref ret i) i))
  (dotimes (i cnt)
    (let* ((x (random size))
	   (y (random size))
	   (buf (aref ret x)))
      (setf (aref ret x) (aref ret y))
      (setf (aref ret y) buf)))
  ret)


(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_x6_iter_60000000.caffemodel")
(let* ((id2 (random 1000)) (id 0.0) (step 0.01) (max 1.0) (min 0.0)
       (min-dif *inf*) (min-dif-id id) buf)
  (do-until-key
   (format t "[param] (db_id, rd_id) = (~A, ~A)~%" id2 id)
   (format t "        (min_dif, rd_id) = (~A, ~A)~%" min-dif min-dif-id)
   (setq id (+ id step))
   (if (> min-dif
	  (setq buf (apply 'max (map cons 'abs (test-redundancy-ik :id id2 :redundancy id)))))
       (setq min-dif buf min-dif-id id))
   (cond
    ((> id max)
     (test-redundancy-ik :id id2 :redundancy min-dif-id)
     (return-from nil (print min-dif))))
   (if (or (> id max) (< id min)) (setq step (* -1 step)))))

(caffe::gen-test-net :netproto "redundancy_ik_net_predict.prototxt"
		     :caffemodel "redundancy_ik_net_iter_50000000.caffemodel")

(caffe::calc-forward-double
 :isize 7 :osize 7
 :num 1 :channels 7 :width 1 :height 1
 :input (float-vector -0.105524 0.0936122 0.250577 -0.603332 0.946547 1.00249
		      0))

(dotimes (i *sample-points*)
  (print (norm (subseq *ef-coords* (* i 6) (* (+ i 1) 6)))))

roseus learn.l "(progn (ik-learn) (exit))" > log.learn 2>&1
