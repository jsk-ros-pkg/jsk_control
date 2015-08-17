
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
  (append *rarm-links* (send *robot* :hand :rarm :links)))

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

(objects (append
	  ;; (list *arrow*)
	  *stick-man-obj*
	  *rarm-links*
	  (send *robot* :hand :rarm :links)))


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

(defun draw-candidates
  (&key (target
	 (setq *input*
	       (progn
		 (mapcar #'(lambda (j)
			     (send j :joint-angle
				   (+ (send j :min-angle)
				      (* (random 1.0)
					 (- (send j :max-angle) (send j :min-angle))))))
			 (butlast (send *robot* :rarm :joint-list)))
		 (send-all (send *robot* :links) :worldcoords)
		 (concatenate
		  float-vector
		  (scale 1e-3
			 (send (send (car (send *robot* :links)) :copy-worldcoords)
			       :difference-position (send *robot* :rarm :end-coords :copy-worldcoords)))
		  (send (send (car (send *robot* :links)) :copy-worldcoords)
			:difference-rotation (send *robot* :rarm :end-coords :copy-worldcoords))))))
	(cnt 0)
	(step 0.01)
	(draw-step 0.03)
	(draw-cnt 0)
	(thre (float-vector 0.05 0.05 0.05 0.5 0.5 0.5))
	(draw? t)
	;;
	(total-cnt 0)
	(update-cnt 0)
	objects-buf
	)
  (setq *input* target)
  ;; (send *irtviewer* :objects nil)
  (let* ((_c (make-coords :pos (scale 1e+3 (subseq *input* 0 3))
			  :rot (matrix-exponent (subseq *input* 3 6))))
	 (c (send (send (car (send *robot* :links)) :copy-worldcoords)
		  :transform _c :local))
	 (joint-list (butlast (send *robot* :rarm :joint-list)))
	 ret)
    (send *arrow* :newcoords (send c :copy-worldcoords))
    (redundancy-ik-search :target *input*)
    ;; (setq objects-buf (send *irtviewer* :objects))
    ;; (send *irtviewer* :objects (gen-stick-man))
    (send *viewer* :draw-objects)
    ;; (draw-perspective)
    (while (< cnt 1.0)
      (let* ((input (concatenate float-vector *input* (float-vector cnt)))
	     (output
	      (caffe::calc-forward-double
	       :isize (length input) :osize (length joint-list)
	       :num 1 :channels (length input) :width 1 :height 1
	       :input input))
	     (update? t)
	     )
	;;
	(map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
					:relative t))
	     joint-list output)
	(send-all (send *robot* :links) :worldcoords)
	(let* ((dif (instantiate float-vector 6)))
	  (normal-check-ik-thre output dif)
	  ;; (print dif)
	  (cond
	   ((minusp (apply 'max (coerce dif cons)))
	    (incf update-cnt)
	    (push (concatenate float-vector output (float-vector cnt)) ret))
	   (t (setq update? nil))))
	(incf total-cnt)
	;;
	(cond
	 ((and draw? (or update? (> draw-cnt draw-step)))
	  (draw-arm-line (if update? (float-vector 1 0 0) (float-vector 0 1 0))
			 (if update? 1.0 1.0))
	  (setq draw-cnt 0)
	  ))
	)
      (setq cnt (+ cnt step))
      (setq draw-cnt (+ draw-cnt step)))
    ;;
    (send *viewer* :viewsurface :color (float-vector 0 0 0))
    (send *viewer* :viewsurface :string
	  10 (- (send *viewer* :viewsurface :height) 10)
	  (format nil "~A/~A reachable" update-cnt total-cnt))
    (send *arrow* :draw-on :flush nil :color (float-vector 0 0 1) :size 200 :width 40)
    ;;
    (send *viewer* :viewsurface :flush)
    ;; (send *irtviewer* :objects objects-buf)
    ret))

(unix::system "rm -rf gather_log")
(unix::system "mkdir gather_log")
(dotimes (i 10)
  (send *irtviewer* :objects *arm-objects*)
  (unix::system (format nil "mkdir gather_log/~A" i))
  (let* ((d (draw-candidates :step 0.001)) (j -1) prev)
    (send *viewer* :viewsurface :write-to-image-file
	  (format nil "gather_log/~A/all_candidates.jpg" i))
    (unix:sleep 2)
    ;; (read-line)
    (send *irtviewer* :objects (cons *arrow* *arm-objects*)) ;;*stick-man-obj*))
    (dolist (output d)
      (if (not prev) (setq prev (map float-vector '(lambda (d) (+ d (rad2deg 5))) output)))
      (cond
       ((< (norm (v- prev output)) (deg2rad 5))
	(incf j) 'nop)
       (t
	(setq prev output)
	(format t "~A/~A~%" (incf j) (length d))
	(map cons '(lambda (j av) (send j :joint-angle (- (rad2deg av) (send j :joint-angle))
					:relative t))
	     (butlast (send *robot* :rarm :joint-list)) output)
	(send *viewer* :draw-objects :flush nil)
	(draw-arm-line)
	;;
	(send *viewer* :viewsurface :color (float-vector 0 0 0))
	(send *viewer* :viewsurface :string
	      10 (- (send *viewer* :viewsurface :height) 10)
	      (format nil "redundancy: ~A" (aref output (- (length output) 1))))
	;;
	(send *viewer* :viewsurface :flush)
	(send *viewer* :viewsurface :write-to-image-file
	      (format nil "gather_log/~A/~A.jpg" i j))
	;; (read-line)
	))))
  )
