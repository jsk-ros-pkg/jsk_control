#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(require "nlopt.l")
(defvar *robot* (hrp2-simple-detail))

(defvar *move-target* (send *robot* :rarm :end-coords))
(defvar *links* (send *robot* :link-list (send *move-target* :parent)))
(defvar *target-coords*
  (make-coords :pos #F(700 0 1000) :rot (rotation-matrix (deg2rad -90) :x)))

(pickview :no-menu t)
(objects *robot*)

(defun-c-callable endcoords-diff ((v1 :string) (v2 :string)) :integer
  (let (buf)
    (map cons #'(lambda (l v) (send (send l :joint) :joint-angle (rad2deg v)))
	 *links* v1)
    (send *pickview* :draw-objects)
    (x::window-main-one)
    (setq buf (concatenate float-vector
			   (scale 1e-3
				  (send *move-target* :difference-position *target-coords*))
			   (send *move-target* :difference-rotation *target-coords*)))
    (setf (aref v2 0) (norm2 buf)))
  0)

(defun-c-callable jacobian ((v1 :string) (v2 :string)) :integer
  (let (dx J buf)
    (map cons #'(lambda (l v) (send (send l :joint) :joint-angle (rad2deg v)))
	 *links* v1)
    (send *pickview* :draw-objects)
    (x::window-main-one)
    (setq J (send *robot* :calc-jacobian-from-link-list
		  *links*
		  :move-target *move-target*
;		  :transform-coords (make-coords)
		  :translation-axis '(t)
		  :rotation-axis '(t)))
    (setq dx
	  (concatenate float-vector
		       (scale 1e-3
			      (send *move-target* :difference-position *target-coords*))
		       (send *move-target* :difference-rotation *target-coords*)))
    (my-copy-matrix (transform (transpose J) (scale -2 dx)) v2)
    )
  0)

(defun ik-test
  (&key
   (target-limb :rarm)
   (move-target(send *robot* target-limb :end-coords))
   (link-list (send *robot* :link-list (send move-target :parent)))
   (target-coords
    (make-coords :pos #F(700 0 1000)
		 :rot (rotation-matrix (deg2rad -90) :x)))
   (eus? nil) (reset? t) (alg 9)
   (x0
    (progn
      (cond (reset? (send *robot* :reset-pose) (send *pickview* :draw-objects)))
      (coerce
       (mapcar
	#'(lambda (l) (deg2rad (send (send l :joint) :joint-angle)))
	*links*)
       float-vector))))
  (setq *target-coords* target-coords)
  (setq *links* link-list)
  (setq *move-target* move-target)
  (cond
   (eus?
    (send *robot* target-limb :inverse-kinematics *target-coords*
	  :debug-view :no-message
	  :link-list *links* :move-target *move-target*))
   (t (nlopt-optmize :alg alg
		     :x0 x0
		     :x-min (coerce
			     (mapcar
			      #'(lambda (l) (deg2rad (send (send l :joint) :min-angle)))
			      *links*)
			     float-vector)
		     :x-max (coerce
			     (mapcar
			      #'(lambda (l) (deg2rad (send (send l :joint) :max-angle)))
			      *links*)
			     float-vector)
		     :f 'endcoords-diff :df 'jacobian
		     :g 'nop :dg 'nop
		     :h 'nop :dh 'nop
		     :m-x (length *links*)
		     :m-g 0 :m-h 0))))

(ik-test)
#|
(defun print-link-cascade
  (&key
   (link (send *robot* :rarm :root-link))
   (joint (send link :joint))
   (childs (send link :child-links))
   (depth 0))
  (dotimes (i depth) (format t " "))
  (format t "~A (+- ~A~%" (send link :name) (send joint :name))
  (cond
   ((null childs) nil)
   (t (mapcar
       #'(lambda (c)
	   (print-link-cascade :link c :depth (+ depth 1)))
       childs))))