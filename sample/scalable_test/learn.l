#!/usr/bin/env roseus

(require "package://eus_caffe/euslisp/eus-caffe.l")
(require "package://eus_caffe/euslisp/eus-caffe-db.l")
(require "package://eus_caffe/euslisp/eus-log.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
(require "models/arrow-object.l")
(require "package://eus_caffe/sample/scalable_test/gen-solver.l")

(defclass vector-coords
  :super propertied-object
  :slots (coords mode))
(defmethod vector-coords
  (:init (c
          &key ((:mode m) :coords))
         (setq coords c) (setq mode m))
  (:worldcoords
   nil
   (let* ((c (cond ((functionp coords) (funcall coords))
                   (t coords))))
     (cond
      ((not (find-method c :worldcoords))
       (warning-message 1 "[coords2vector] invalid coords format ~A~%" coords)
       (throw :vector-coords-exception t)))
     (send c :worldcoords)))
  ;;
  (:vector
   (&key (org-coords (make-coords)))
   (let* ((c (send self :worldcoords)) ret)
     (cond
      ((eq mode :coords)
       (let* (pos rot)
         (setq pos (send (send org-coords :worldcoords)
                         :difference-position c))
         (setq rot (send (send org-coords :worldcoords)
                         :difference-rotation c))
         (setq ret (concatenate float-vector (scale 1e-3 pos) rot))))
      ((eq mode :pos)
       (let* (pos)
         (setq pos (send (send org-coords :worldcoords)
                         :difference-position
                         c))
         (setq ret (scale 1e-3 pos))))
      (t
       (warning-message 1 "[coords2vector] invalid mode ~A~%" mode)
       (throw :vector-coords-exception t)))
     ret))
  (:length
   (&optional vec ret)
   (setq ret
         (cond ((eq mode :coords) 6) ((eq mode :pos) 3)
               (t (warning-message 1 "[coords-vector] mode invalid~A~%" mode)
                  (throw :vector-coords-exception t))))
   (cond
    ((and vec (not (eq ret (length vec))))
     (warning-message 1 "[coords-vector] invalid length~A~%" vec)
     (throw :vector-coords-exception t)))
   ret)
  (:coords-from-vector
   (vec &key (org-coords (make-coords)) c)
   ;; check length
   (send self :length vec)
   ;;
   (cond
    ((eq mode :coords)
     (setq c (make-coords
              :pos (scale 1e+3 (subseq vec 0 3))
              :rot (matrix-exponent (subseq vec 3 6))))
     (setq c (send (send org-coords :copy-worldcoords)
                   :transform c :local))
     c)
    ((eq mode :pos)
     (setq c (make-coords :pos (scale 1e+3 vec)))
     (setq c (send (send org-coords :copy-worldcoords)
                   :transform c :local))
     c))
   c)
  (:nomethod
   (&rest args)
   (send* (send self :worldcoords) (car args) (cdr args)))
  )

(defvar *robot* (hrp2jsknt))
(defvar *joint-list1* (append (butlast (send *robot* :rarm :joint-list))
                              (send *robot* :torso :joint-list)))
;; (setq *joint-list1* (list (send *robot* :rarm :elbow-p) (send *robot* :rarm :shoulder-p)))
(defvar *joint-list2* (append (butlast (send *robot* :rarm :joint-list))
                              (butlast (send *robot* :larm :joint-list))
                              (send *robot* :torso :joint-list)))
(defvar *joint-list3* (append (butlast (send *robot* :rarm :joint-list))
                              (butlast (send *robot* :larm :joint-list))
                              (butlast (send *robot* :rleg :joint-list))
                              (butlast (send *robot* :lleg :joint-list))
                              (send *robot* :torso :joint-list)))

(defvar *ef-coords* (list (instance vector-coords :init (send *robot* :rarm :end-coords))))
;; (defvar *ef-coords-length* (list 6))
(defvar *arrow* (mapcar '(lambda (ec) (arrow)) *ef-coords*))
(defvar *root-link* (car (send *robot* :links)))

(setq *joint-list1* *joint-list2*)
(setq *ef-coords*
      (mapcar
       '(lambda (c mode) (instance vector-coords :init c :mode mode))
       (list (send *robot* :larm :end-coords)
             (send *robot* :rarm :end-coords)
             ;; (send *robot* :lleg :end-coords)
             ;; '(lambda (&rest args)
             ;;    (make-coords :pos (send *robot* :centroid)))
             )
       (list :coords :coords :coords :pos)))
(setq *arrow* (mapcar '(lambda (ec) (arrow)) *ef-coords*))

(objects (flatten (list (car (send *robot* :links))
                        (reverse (send-all *joint-list3* :child-link))
                        *arrow*)))

(defvar *sample-points* ;;96)
  (let* ((org
          (min
           (round 1e+6)
           (expt (max
                  (min (round (expt 1e+6 (/ 1.0 (length *joint-list1*))))
                       4) 2) (length *joint-list1*))))
         (sep 32))
    (if (not (zerop (mod org sep)))
        (max 1 (round (* (ceiling (/ (* 1.0 org) sep)) sep)))
      org)))

(defvar *angle-vector-map*)
(defvar *ef-coords-map*)

(defun get-vector-from-ef-coords
  (&key
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (org-coords (send root-link :worldcoords)))
  (apply 'concatenate
         (cons float-vector
               (send-all ef-coords :vector :org-coords org-coords))))

(defun get-ef-coords-from-vector
  (&key
   (ef-coords *ef-coords*)
   (vector (get-vector-from-ef-coords))
   (root-link *root-link*)
   (org-coords (send root-link :worldcoords))
   (i 0) len
   ret)
  (dolist (efc ef-coords)
    (setq len (send efc :length))
    (push (send efc :coords-from-vector (subseq vector i (+ i len))
                :org-coords org-coords) ret)
    (setq i (+ i len)))
  (reverse ret))

(defun shuffle-random-vector-map
  (&key
   (sample-points *sample-points*)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   ;; (ef-coords-length *ef-coords-length*)
   (av-map *angle-vector-map*)
   (cc-map *ef-coords-map*)
   ;;
   (len (length (get-vector-from-ef-coords))) ;;(apply '+ ef-coords-length))
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
        (dotimes (j len)
          (swap cc-map (+ j (* i len)) (+ j (* r len))))))))

(defun gen-ik-learning-data
  (&key (sample-points *sample-points*)
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        ;; (ef-coords-length *ef-coords-length*)
        (sep (round (expt sample-points (/ 1.0 (length joint-list)))))
        (root-link *root-link*) ;;(car (send *robot* :links)))
        (i 0)
        (draw? nil)
        (surfix "")
        (len (length (get-vector-from-ef-coords)))
        )
  ;;
  (cond
   ((< sep 2)
    (warning-message 3 "large search space ~A^~A~% -> random DB~%"
                     (length joint-list) sep)
    (gen-random-ik-learning-data
     :sample-points sample-points
     :joint-list joint-list
     :ef-coords ef-coords
     :sep sep
     :root-link root-link
     :i i
     :draw? draw?
     :surfix surfix))
   (t
    ;;
    (setq *sample-points* sample-points)
    (if (not (eq (length *angle-vector-map*)
                 (* (length joint-list) sample-points)))
        (setq *angle-vector-map*
              (instantiate float-vector
                           (* (length joint-list) sample-points))))
    (if (not (eq (length *ef-coords-map*)
                 (* len sample-points)))
        (setq *ef-coords-map*
              (instantiate float-vector (* len sample-points))))
    (labels ((itter
              (jlist)
              (cond
               ((>= i sample-points)
                'nop)
               ((null jlist)
                (let* (val)
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
                  ;;
                  (setq val (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
                  (dotimes (j len)
                    (setf (aref *ef-coords-map* (+ j (* i len))) (aref val j)))
                  (setq i (+ i 1))
                  (if (and (> sample-points 100)
                           (zerop (mod i (round (/ sample-points 100)))))
                      (warning-message 2 "~A%~%" (/ i (round (/ sample-points 100)))))
                  ))
               (t
                (dotimes (j sep)
                  (send (car jlist) :joint-angle
                        (-
                         (+ (send (car jlist) :min-angle)
                            (* (/ j (- sep 1.0))
                               (- (send (car jlist) :max-angle)
                                  (send (car jlist) :min-angle))))
                         (send (car jlist) :joint-angle))
                        :relative t)
                  (itter (cdr jlist))))
               )))
      (warning-message 1 "generate sep DB ~A^~A~%" (length joint-list) sep)
      (itter joint-list)
      ;; fill random
      (warning-message 1 "generate random DB ~A~%" (- sample-points i))
      (gen-random-ik-learning-data
       :sample-points sample-points
       :joint-list joint-list
       :ef-coords ef-coords
       :sep sep
       :root-link root-link
       :i i
       :draw? draw?
       :dump? nil)
      ;;
      (warning-message 1 "shuffling DB~%")
      (shuffle-random-vector-map)
      (warning-message 1 "write to file .~A~%" surfix)
      (dump-ik-learning-data :surfix surfix)
      ))))

(defun gen-random-angle-vector
  (&optional (joint-list *joint-list1*))
  (dolist (j joint-list)
    (send j :joint-angle
          (-
           (+ (send j :min-angle)
              (* (random 1.0)
                 (- (send j :max-angle) (send j :min-angle))))
           (send j :joint-angle))
          :relative t)))

(defun gen-random-ik-learning-data
  (&key (sample-points *sample-points*)
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        (sep (round (expt sample-points (/ 1.0 (length joint-list)))))
        (root-link *root-link*)
        ;; (root-link (car (send *robot* :links)))
        (i 0)
        (draw? nil)
        (dump? t)
        (surfix (if dump?
                    (format nil ".~A" (unix-command-line "basename `pwd`"))))
        (gen-random-angle-vector-func 'gen-random-angle-vector)
        ;;
        (len (length (get-vector-from-ef-coords)))
        &allow-other-keys
        )
  (setq *sample-points* sample-points)
  (if (not (eq (length *angle-vector-map*)
               (* (length joint-list) sample-points)))
      (setq *angle-vector-map*
            (instantiate float-vector
                         (* (length joint-list) sample-points))))
  (if (not (eq (length *ef-coords-map*)
               (* len sample-points)))
      (setq *ef-coords-map*
            (instantiate float-vector (* len sample-points))))
  ;;
  (let* (val)
    (dotimes (_i sample-points)
      (if (>= i sample-points) (return-from nil nil))
      (if (and (> sample-points 100)
               (zerop (mod _i (round (/ sample-points 100)))))
          (warning-message 3 "~A%~%" (/ _i (round (/ sample-points 100)))))
      ;; (dolist (j joint-list)
      ;;   (send j :joint-angle
      ;;         (-
      ;;          (+ (send j :min-angle)
      ;;     (* (random 1.0)
      ;;        (- (send j :max-angle) (send j :min-angle))))
      ;;          (send j :joint-angle))
      ;;         :relative t)
      ;;   )
      (if (functionp gen-random-angle-vector-func)
          (funcall gen-random-angle-vector-func joint-list)
        (warning-message 1 "invalid gen-random-angle-vector-func~%"))
      (send-all (send-all joint-list :child-link) :worldcoords)
      (send-all ef-coords :worldcoords)
      (dotimes (j (length joint-list))
        (setf (aref *angle-vector-map* (+ j (* i (length joint-list))))
              (deg2rad (send (nth j joint-list) :joint-angle))))
      ;;
      (setq val (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
      (dotimes (j len)
        (setf (aref *ef-coords-map* (+ j (* i len))) (aref val j)))
      ;;
      (incf i)
      )
    (if dump? (dump-ik-learning-data :surfix surfix))
    ))

;; gen-random-ik-learning-data-with-thre :sample-points 3 :thre 0.3 :max-time 5
;; check-test-data 0
;; ik-random-test-min :revert? t :random-angle (send-all *joint-list1* :joint-angle)
(defun gen-random-ik-learning-data-with-thre
  (&rest args
         &key
         (draw? t)
         (thre 0.3)
         (start-time (car (unix::gettimeofday)))
         (max-time (* 15 60))
         (_max-time (+ start-time max-time))
         (time-over-flag nil)
         (overthre-cnt 0)
         (nop-cnt 1)
         ret
         &allow-other-keys)
  (smart-initialize-net)
  (cpplog::change-output-stream "/dev/null")
  (setq
   ret
   (apply 'gen-random-ik-learning-data
          :gen-random-angle-vector-func
          #'(lambda (joint-list)
              (gen-random-angle-vector joint-list)
              (cond
               (time-over-flag
                (incf nop-cnt);; (print 'nop)
                )
               ((< _max-time (car (unix::gettimeofday)))
                (setq time-over-flag t)
                (warning-message 1 "time exceeded ~A~%" max-time))
               (t
                (let* ((dif))
                  (setq dif (apply 'ik-random-test-min
                                   :redundancy-step 0.1
                                   :revert? t :initialize? nil
                                   :draw? draw? args))
                  (while (and
                          (not time-over-flag)
                          (not (setq time-over-flag (< _max-time (car (unix::gettimeofday)))))
                          (< (cdr (assoc :state-diff-norm dif)) thre))
                    (if draw? (warning-message 6 "random av rejected: ~A < ~A~%" dif thre))
                    ;; (gen-random-angle-vector joint-list)
                    (setq dif (apply 'ik-random-test-min
                                     :redundancy-step 0.1
                                     :revert? t :initialize? nil
                                     :draw? draw? args))
                    )
                  (cond
                   ((not time-over-flag) ;; (>= (cdr (assoc :state-diff-norm dif)) thre)
                    (incf overthre-cnt)
                    (if draw?
                        (warning-message 5 "random av accepted~A: ~A < ~A~%" overthre-cnt dif thre)))
                   ;; (t (incf nop-cnt))
                   )
                  ))))
          args))
  (warning-message 1 "[gen-random-ik-learning-data-with-thre] thre: ~A, over: ~A, nop: ~A~%"
                   thre overthre-cnt nop-cnt)
  (cpplog::change-output-stream "")
  ret
  )

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
  ;;      :size (length *angle-vector-map*)
  ;;      :data *angle-vector-map*)
  ;; (caffe::reset-memory-layer :name "target"
  ;;      :size (length *ef-coords-map*)
  ;;      :data *ef-coords-map*)
  (cond
   ((or (zerop (length *ef-coords-map*))
        (zerop (length *angle-vector-map*)))
    (warning-message 1 "zero training DB~%"))
   (t
    (caffe::learn
     :solver solver
     :solverstate solverstate
     :size *sample-points*
     :ddata *ef-coords-map*
     :idata *angle-vector-map*
     ))))

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
  (solver
   &key
   (best-path solver)
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
  ;;
  (warning-message 3 (format nil "~A~%" solver-lnk))
  (if (string-equal solver "ik_solver.best.prototxt")
      (print 'skip)
    (unix::system solver-lnk))
  (warning-message 3 (format nil "~A~%" caffe-lnk))
  (unix::system caffe-lnk)
  (warning-message 3 (format nil "~A~%" solverstate-lnk))
  (unix::system solverstate-lnk)
  )

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
  (&rest args &key (thre) &allow-other-keys)
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
    (if (numberp thre)
        (gen-random-ik-learning-data-with-thre :draw? nil :thre thre)
      (gen-random-ik-learning-data))
    (warning-message 3 "update solver~%")
    (let* ((p (apply 'update-solver :input "ik_solver.best.prototxt" args)))
      (unix::system (format nil "ln -snf ~A ik_solver.best.prototxt" p)))
    (warning-message 3 "start learning ... ~%")
    (apply 'ik-learn :solver config :solverstate state args)
    ;; (ik-learn :solver solver :solverstate state)
    (warning-message 3 "done~%")
    (link-best-settings "ik_solver.best.prototxt")
    (warning-message 3 "evaluate network~%")
    (cpplog::change-output-stream "/dev/null")
    (let* ((ret (ik-random-test-min-average :draw? nil)))
      (cpplog::change-output-stream "")
      ret)
    ))

(defun ik-learn-best-loop
  (&rest args
         &key
         (lp 10)
         ;;
         (base_lr-gain 1.0)
         (eq-scale-thre 0.01)
         (base_lr-gain-update-scale 1.1)
         (vain-cnt 0)
         (vain-cnt-max 5) prev now
         (thre
          (progn
            (cpplog::change-output-stream "/dev/null")
            (let* ((ret (ik-random-test-min-average :draw? nil)))
              (cpplog::change-output-stream "")
              ret)))
         ;;
         &allow-other-keys)
  (let* ((ret))
    (push thre ret)
    (dotimes (i lp)
      (push (apply 'ik-learn-best :base_lr-gain base_lr-gain :thre (car ret) args) ret)
      (setq prev now)
      (setq now (car ret))
      ;; update base_lr-gain
      (cond
       ((or (not (numberp now)) (not (numberp prev)))
        'skip)
       ((eps= now prev (* now eq-scale-thre))
        ;;(setq base_lr-gain (* base_lr-gain base_lr-gain-update-scale))
        (setq base_lr-gain base_lr-gain-update-scale)
        (incf vain-cnt)
        )
       ((eps< now prev (* now eq-scale-thre))
        (setq base_lr-gain 1.0)
        (decf vain-cnt)
        'nop
        )
       ((eps> now prev (* now eq-scale-thre))
        ;; (setq base_lr-gain (/ base_lr-gain base_lr-gain-update-scale))
        (setq base_lr-gain (/ 1.0 base_lr-gain-update-scale))
        (incf vain-cnt)
        ))
      (warning-message
       3 (format nil "~A~%" (unix-command-line "ls -l ik.best.caffemodel | sed -e \"s/ /\\\n/g\" | tail -1")))
      (warning-message
       3 (format nil "~A~%" (unix-command-line "ls -l ik.best.solverstate | sed -e \"s/ /\\\n/g\" | tail -1")))
      (warning-message 3 "update base_lr-gain: ~A, vain: ~A, now: ~A~%"
                       base_lr-gain vain-cnt now)
      ;; convergence
      (cond
       ((>= vain-cnt vain-cnt-max)
        (warning-message 3 "loop exceeded, ~A > ~A~%"
                         vain-cnt vain-cnt-max)
        (return-from nil nil)))
      )
    (setq *ret* ret)
    ret))

(defun check-test-data
  (&optional
   (i (random *sample-points*)) ;;0)
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (len (length (get-vector-from-ef-coords)))
   (input (instantiate float-vector len))
   (output (instantiate float-vector (length joint-list)))
   c c-buf
   )
  (setq c-buf
        (get-ef-coords-from-vector
         :ef-coords ef-coords
         :vector (subseq *ef-coords-map* (* i len) (* (+ i 1) len))
         :root-link root-link))
  (dotimes (j (length joint-list))
    (setf (aref output j)
          (aref *angle-vector-map* (+ (* i (length joint-list)) j))))
  (map cons #'(lambda (j a) (send j :joint-angle
                                  (- (rad2deg a) (send j :joint-angle))
                                  :relative t))
       joint-list output)
  (send-all (send-all joint-list :child-link) :worldcoords)
  (send-all ef-coords :worldcoords)
  (mapcar
   '(lambda (ar c)
      (send ar :newcoords (send c :copy-worldcoords)))
   *arrow* c-buf)
  (send *viewer* :draw-objects :flush nil)
  (send *viewer* :viewsurface :flush)
  (print (apply 'concatenate (cons float-vector (flatten (mapcar '(lambda (ec c) (list (send c :difference-position ec) (send c :difference-rotation ec))) (send-all ef-coords :worldcoords) c-buf)))))
  )

(defun smart-initialize-net
  nil
  (let* ((caffemodel (unix-command-line "ls -t | grep -e \"caffemodel$\" | head -n 1")))
    (warning-message 6 "load caffe model ~A~%" caffemodel)
    (caffe::gen-test-net :netproto "predict_ik_net.prototxt"
                         :caffemodel caffemodel))
  )

(defun ik-random-test
  (&key (calc-forward 'caffe::calc-forward-double) ;;'caffe::memory-calc-forward)
        (draw? t) (revert? nil)
        tm
        (joint-list *joint-list1*)
        (ef-coords *ef-coords*)
        (root-link *root-link*)
        (random-angle
         (mapcar '(lambda (j) (send j :joint-angle
                                    (-
                                     (+ (send j :min-angle)
                                        (* (random 1.0)
                                           (- (send j :max-angle)
                                              (send j :min-angle))))
                                     (send j :joint-angle))
                                    :relative t))
                 joint-list))
        (update-coords
         (progn (send-all (send-all joint-list :child-link) :worldcoords)
                (send-all ef-coords :worldcoords)))
        (c (send-all (send-all ef-coords :worldcoords) :copy-worldcoords))
        (b (send root-link :copy-worldcoords))
        (av (instantiate float-vector (length joint-list)))
        (input (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
        (initialize? (smart-initialize-net))
        (redundancy;; (float-vector 0.5))
         (fill
          (instantiate float-vector (- (caffe::get-blob-count "input") (length input)))
          0.5))
        &allow-other-keys
        )
  (setq input (concatenate float-vector input redundancy))
  (setq
   tm
   (bench2
    (apply calc-forward
           (list :isize (length input) :input input
                 :osize (length av) :output av))
    ))
  (map cons #'(lambda (j a) (send j :joint-angle
                                  (- (rad2deg a) (send j :joint-angle))
                                  :relative t))
       joint-list av)
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
   (cons :state (copy-seq av))
   (cons :redundancy (copy-seq redundancy))
   (cons :task-diff
         (apply 'concatenate (cons float-vector (flatten (mapcar '(lambda (ec c) (list (scale 1e-3 (send c :difference-position ec)) (send c :difference-rotation ec))) (send-all ef-coords :worldcoords) c))))
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
   (cons :revert?
         (if revert? (mapcar '(lambda (j av) (send j :joint-angle av)) joint-list random-angle)))
   ))

(defun ik-random-test-min
  (&rest
   args
   &key
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (random-angle
    (mapcar '(lambda (j) (send j :joint-angle
                               (-
                                (+ (send j :min-angle)
                                   (* (random 1.0)
                                      (- (send j :max-angle)
                                         (send j :min-angle))))
                                (send j :joint-angle))
                               :relative t))
            joint-list))
   (update-coords
    (progn (send-all (send-all joint-list :child-link) :worldcoords)
           (send-all ef-coords :worldcoords)))
   (c (send-all (send-all ef-coords :worldcoords) :copy-worldcoords))
   (b (send root-link :copy-worldcoords))
   (av (instantiate float-vector (length joint-list)))
   (input (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
   (initialize? (smart-initialize-net))
   (redundancy
    (fill
     (instantiate float-vector (- (caffe::get-blob-count "input") (length input)))
     0.0))
   (step-max 300)
   (redundancy-step
    (/ 1.0 (expt step-max (/ 1.0 (length redundancy)))))
   (revert? nil)
   buf ret
   &allow-other-keys
   )
  (labels ((itter
            (depth)
            (cond
             ((zerop depth)
              ;; (print redundancy)
              (push (apply 'ik-random-test :random-angle random-angle :update-coords update-coords :c c :b b :av av :input input :redundancy redundancy :initialize? nil :revert? revert? args) buf))
             (t
              (dotimes (j (round (/ 1.0 redundancy-step)))
                (setf (aref redundancy (- depth 1)) (* j redundancy-step))
                (itter (- depth 1)))))))
    (itter (length redundancy)))
  ;; (dotimes (i (length redundancy))
  ;;   (dotimes (j (round (/ 1.0 redundancy-step)))
  ;;     (setf (aref redundancy i) (* j redundancy-step))
  ;;     (push (apply 'ik-random-test :random-angle random-angle :update-coords update-coords :c c :b b :av av :input input :redundancy redundancy :initialize? nil args) buf)))
  (setq *ret* buf)
  (setq ret (car (sort buf #'(lambda (a b) (< (cdr (assoc :state-diff-norm a))
                                              (cdr (assoc :state-diff-norm b)))))))
  (if (not revert?)
      (apply 'ik-random-test :random-angle random-angle :update-coords update-coords :c c :b b :av av :input input :redundancy (cdr (assoc :redundancy ret)) :initialize? nil :revert? revert? args))
  ret
  )

(defun ik-random-test-min-average
  (&rest args &key (lp 100) &allow-other-keys)
  (let* ((ret))
    (smart-initialize-net)
    (dotimes (i lp)
      (push (apply 'ik-random-test-min :initialize? nil args) ret))
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
