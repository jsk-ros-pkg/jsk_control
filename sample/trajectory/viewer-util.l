
(send *irtviewer* :resize 300 300)
(send *irtviewer* :change-background (float-vector 1 1 1))
(send *irtviewer* :set-val 'x::draw-origin nil)
(send *irtviewer* :set-val 'x::draw-floor nil)

(send (car (send *robot* :links)) :transform
      (send (send (car (send *robot* :rarm :links)) :copy-worldcoords)
	    :transformation
	    (make-coords :pos (float-vector 500 500 500)))
      :local)

(defvar *arm-objects*
  (append (send-all *rarm-joints1* :child-link)
	  (send *robot* :hand :rarm :links)))

(defmethod joint
  (:worldcoords
   nil
   (if (and parent-link (find-method parent-link :copy-worldcoords))
       (send (copy-object (send parent-link :worldcoords))
             :transform default-coords)))
  (:worldpos
   nil (send (send self :worldcoords) :worldpos))
  (:worldrot
   nil (send (send self :worldcoords) :worldrot))
  )

(defun gen-stick-man
  (&key
   (joint-list (send *robot* :rarm :joint-list))
   (cy-data
    (list (list (cons :rpy #\r)
		(cons :axis (float-vector 1 0 0))
		(cons :color (float-vector 1 0 0))
		(cons :rotate (list (deg2rad 90) :y)))
	  (list (cons :rpy #\p)
		(cons :axis (float-vector 0 1 0))
		(cons :color (float-vector 0 1 0))
		(cons :rotate (list (deg2rad 90) :x)))
	  (list (cons :rpy #\y)
		(cons :axis (float-vector 0 0 1))
		(cons :color (float-vector 0 0 1))
		(cons :rotate (list (deg2rad 90) :z)))))
   )
  (append
   (mapcar
    #'(lambda (joint)
	(let* ((axis (send joint :get-val 'axis))
	       ;; (rpy (aref (cadr (reg-split "-" (format nil "~A" (send joint :name)))) 0))
	       (cy (make-cylinder 25 100))
	       (data ;;(find-if #'(lambda (d) (eq rpy (cdr (assoc :rpy d)))) cy-data))
		(car (sort cy-data #'(lambda (a b) (< (norm (v- axis (cdr (assoc :axis a))))
						      (norm (v- axis (cdr (assoc :axis b)))))))))
	       )
	  (send cy :set-color
		(if (eq joint (car (last joint-list))) (float-vector 1 0.7 0)
		  (cdr (assoc :color data))))
	  (apply #'send (append (list cy :rotate) (cdr (assoc :rotate data))))
	  (send cy :translate (scale -1 (send cy :world-centroid)) :world)
	  (send cy :transform (send (send joint :worldcoords) :copy-worldcoords) :world)
	  (send (send joint :child-link) :assoc cy)
	  ;; (send cy :translate
	  ;; (v- (send joint :worldpos)
	  ;; (send cy :world-centroid))
	  ;; :world)
	  cy
	  ))
    joint-list)
   ;;
   (flatten
    (mapcar
     #'(lambda (l-bef l-aft)
	 (if (< (norm (v- (send l-bef :worldpos) (send l-aft :worldpos))) 1e-1)
	     nil
	   (let* ((cy (make-cylinder 10 (norm (v- (send l-aft :worldpos) (send l-bef :worldpos)))))
		  (dir (normalize-vector (v- (send l-aft :worldpos) (send l-bef :worldpos))))
		  )
	     (send cy :set-color #F(0.3 0.3 0.3))
	     (send cy :newcoords
		   (make-coords
		    :pos (copy-seq (send l-bef :worldpos))
		    :rot
		    (matrix-exponent
		     (scale (acos (v. (float-vector 0 0 1) dir))
			    (normalize-vector (v* (float-vector 0 0 1) dir)))
		     )))
	     (send (send l-aft :child-link) :assoc cy)
	     ;; (rodrigues
	     ;; (v* #F(0 0 1) dir)
	     ;; (acos (v. #F(0 0 1) dir)))))
	     cy)))
     (cdr joint-list) joint-list)))
  )
(defvar *stick-man-obj* (gen-stick-man))

(objects (cons *goal* *arm-objects*))
;; (objects (append
;; 	  ;; (list *arrow*)
;; 	  *stick-man-obj*
;; 	  *arm-objects*))

(send *viewer* :draw-objects)

(in-package "GL")
(defmethod glviewsurface
  (:string
   (x y str &optional (fid x:font-courb24)) ;;x::font-helvetica-bold-12))       
   (send self :makecurrent)
   (glMatrixMode GL_PROJECTION)
   (glPushMatrix)
   (send self :2d-mode)
   (unless (eq (get self :glxusexfont) fid)
     (setf (get self :glxusexfont) fid)
     (glxUseXfont fid 32 96 (+ 1000 32)))
   (glRasterPos2i (round x) (- (send self :height) (round y)))
   (glListBase 1000)
   (glCallLists (length str) GL_UNSIGNED_BYTE str)
   (send self :3d-mode)
   (glMatrixMode GL_PROJECTION)
   (glPopMatrix)
   (glMatrixMode GL_MODELVIEW)
   ))
(in-package "USER")

(defun draw-perspective
  (&key
   (vw *viewer*)
   (color (float-vector 0.4 0.4 0.4))
   (org (float-vector 0 0 0))
   (max 5000)
   (step 300)
   (pos 0)
   (p1 (float-vector 0 0 0))
   (p2 (float-vector 0 0 0))
   (p3 (float-vector 0 0 0))
   )
  (send vw :viewsurface :color color)
  (send vw :viewsurface :line-width 1.0)
  (dotimes (i 3)
    ;; (setq pos (* -1 max))
    (setq pos 0)
    (dotimes (j 3) (setf (aref p1 j) 0) (setf (aref p2 j) 0) (setf (aref p3 j) 0))
    ;; (setf (aref p1 i) 1)
    (setf (aref p2 (mod (+ i 1) 3)) max)
    (setf (aref p3 (mod (+ i 2) 3)) max)
    (while (< pos max)
      (setf (aref p1 i) pos)
      ;; (send vw :viewsurface :3d-line (v- p1 p2) (v+ p1 p2))
      ;; (send vw :viewsurface :3d-line (v- p1 p3) (v+ p1 p3))
      (send vw :viewsurface :3d-line p1 (v+ p1 p2))
      (send vw :viewsurface :3d-line p1 (v+ p1 p3))
      (setq pos (+ pos step))))
  ;; (send vw :viewsurface :flush)
  )

(defmethod x::irtviewer
  (:draw-objects
   (&rest args)
   (send x::viewer :viewsurface :makecurrent)
   (apply #'gl::draw-globjects
	  x::viewer x::draw-things :draw-origin x::draw-origin
	  :draw-floor x::draw-floor :flush nil
	  args)
   (draw-perspective)
   (send x::viewer :viewsurface :flush)
   ))

(defun draw-arm-line
  (&optional (color (float-vector 0 1 0)) (width 1.0))
  (send *viewer* :viewsurface :color color)
  (send *viewer* :viewsurface :line-width width)
  (mapcar
   '(lambda (l1 l2)
      (send *viewer* :viewsurface :3d-line
	    (send l1 :worldpos) (send l2 :worldpos)))
   (cdr (send *robot* :rarm :links))
   (send *robot* :rarm :links)))

;; (defun rodrigues
;;   (v rad)
;;   (labels ((cross-matrix
;;             (v)
;;             (let ((x (aref v 0))
;;                   (y (aref v 1))
;;                   (z (aref v 2)))
;;               (make-matrix
;;                3 3
;;                (list (list 0 (* -1 z) y)
;;                      (list z 0 (* -1 x))
;;                      (list (* -1 y) x 0))))))
;;     (let ((vx (cross-matrix (normalize-vector v))))
;;       (reduce
;;        #'m+
;;        (list (unit-matrix 3)
;;              (scale-matrix (sin rad) vx)
;;              (scale-matrix (- 1 (cos rad)) (m* vx vx)))))))


(defun draw-candidate
  (&key
   (id (random 10000))
   (input-path "input_lmdb_traj")
   (target-path "target_lmdb_coords")
   ;;
   (initialize-net? nil)
   (redundancy 0.0)
   (input
    (let* ((ret))
      (if initialize-net?
	  (caffe::gen-test-net :netproto "traj_net_predict.prototxt"
			       :caffemodel "traj_net_NESTEROV_iter_5000000.caffemodel"))
      ;;
      (caffe::db-open :path target-path)
      (caffe::db-read id)
      (setq ret (caffe::db-get-float-data))
      (format t " -- size : ~A~%" (caffe::db-get-size))
      (caffe::db-close)
      ret))
   (thre (float-vector 0.1 0.1 0.1
		       0.15 0.15 0.15 0.15))
   target c
   ;;
   (step 0.05)
   (cnt 0.0)
   ret buf
   ;;
   (draw? t)
   (draw-step (/ 1 step))
   (draw-cnt 0)
   )
  (traj-test :id id :input (setq *input* input) :redundancy 0 :draw? nil)
  (mapcar '(lambda (j1 j2) (send j1 :joint-angle (send j2 :joint-angle)))
	  *rarm-joints1* *rarm-joints2*)
  (send *viewer* :draw-objects :flush nil)
  ;; (dotimes (i (round (/ 1.0 step)))
  (setq buf (traj-test :id id :input input :redundancy redundancy
		       :draw-stick? ;; t
		       (list (map float-vector '(lambda (d) (/ d 255.0))
				  (hls2rgb (* 360 redundancy) 0.5 0.5)))
		       :cnt 30))
  (cond
     ((v< (map float-vector 'abs (cdr (assoc :diff buf))) thre)
      ;; (traj-test :id id :input input :redundancy (* i step))
      (push buf ret)))
  (setq ret (sort ret #'(lambda (a b) (< (norm (cdr (assoc :diff a)))
					 (norm (cdr (assoc :diff b)))))))
  ;;
  (send *viewer* :viewsurface :color (float-vector 0 0 0))
  (send *viewer* :viewsurface :string
	10 (- (send *viewer* :viewsurface :height) 10)
	(format nil "redundancy: ~A" redundancy))
  (send *viewer* :viewsurface :flush)
  ret)

#|

(unix::system "rm -rf gather_log")
(unix::system "mkdir gather_log")
(dotimes (j 10)
  (let* ((td (gather-traj-test :step 0.01 :draw? nil
			       :thre (float-vector 0.15 0.15 0.15
						   0.15 0.15 0.15 0.15)))
	 (i -1))
    (setq td (sort td '(lambda (a b) (< (cdr (assoc :redundancy a))
					(cdr (assoc :redundancy b))))))
    (unix::system (format nil "mkdir gather_log/~A" j))
    (dolist (d td)
      (format t "~A/~A~%" (incf i) (length td))
      (draw-candidate :input (subseq (cdr (assoc :input d)) 0 7)
		      :redundancy (cdr (assoc :redundancy d)))
      (unix:usleep 500000)
      (send *viewer* :viewsurface :write-to-image-file
	    (format nil "gather_log/~A/~A.jpg" j i))
      )))


;;

(send *viewer* :draw-objects :flush nil)
(setq *jerk-objective-scale* 1)
(setq
 ret1
 (redundancy-traj-search
  :draw? nil
  :check-trajectory-args (list :draw-stick? (list (float-vector 1 0 0)))
  :input #f(0.297867 -0.768477 0.587254 0.989254 0.095675 0.588501 0.305343)
  :objective 'jerk-objective
  :inequality-dimension 7))
(send *viewer* :viewsurface :flush)

(send *viewer* :draw-objects :flush nil)
;;
(setq *jerk-objective-scale* -1)
(setq
 ret2
 (redundancy-traj-search
  :draw? nil
  :check-trajectory-args (list :draw-stick? (list (float-vector 0 1 0)))
  :input #f(0.297867 -0.768477 0.587254 0.989254 0.095675 0.588501 0.305343)
  :objective 'jerk-objective
  :inequality-dimension 7))
;;
(send *viewer* :viewsurface :flush)
