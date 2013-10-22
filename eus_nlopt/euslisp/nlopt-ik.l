#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(require "nlopt-object.l")

(defun add-6dof-joint
  (&key
   (robot *robot*)
   (link-list
    (if (atom (car *links*)) (list *links*) *links*))
   (min (float-vector -1500 -1500  -1500 -200 -200 -200))
   (max (float-vector 1500 1500 1500 200 200 200))
   (joint-args (list :min min :max max))
   (joint-class 6dof-joint)
   (vlink (instance bodyset-link :init (make-cascoords)
		    :bodies (list (make-cube 150 10 400))
		    :name 'virtual-link
		    :weight 0 :centroid (float-vector 0 0 0)
		    :inertia-tensor (make-matrix 3 3)))
   (rlink (car (send robot :links))))
  (send-message rlink bodyset-link :add-joint
		(instance* (or joint-class 6dof-joint) :init
			   :child-link robot
			   :parent-link vlink
			   joint-args))
  (send rlink :add-parent-link vlink)
  (send vlink :add-child-links rlink)
  (list
   (cons :6dof-links
	 (mapcar #'(lambda (l) (cons rlink l)) link-list))
   (cons :del-6dof-links
	 (eval
	  (list 'function
		(list 'lambda nil
		      (list 'send rlink :del-joint)
		      (list 'send rlink :del-parent-link)
		      (list 'send vlink :del-child-link rlink)))))))

(defun vector-eus2nlopt
  (vl &key
      (vector-scale 1e-3)
      (scala-scale (deg2rad 1)))
  (apply
   #'concatenate
   (cons
    float-vector
    (mapcar
     #'(lambda (v) (cond ((vectorp v) (scale vector-scale v))
			 (t (list (* scala-scale v)))))
     vl))))

(defun vector-nlopt2eus
  (v &key
     (link-list *all-links*)
     (joint-list (send-all link-list :joint))
     (vector-scale 1e+3)
     (scala-scale (rad2deg 1))
     (index 0) buf ret)
  (mapcar
   #'(lambda (j)
       (setq buf (send j :joint-angle))
       (cond
	((vectorp buf)
	 (scale vector-scale (subseq v index (setq index (+ index (length buf))))))
	(t (* scala-scale (aref v (- (incf index) 1))))))
   joint-list))

(defun matrix-set
  (mom sun x y
       &optional
       (width (cdr (assoc 'dim1 (send sun :slots))))
       (height (cdr (assoc 'dim0 (send sun :slots)))))
  (dotimes (dy height)
    (dotimes (dx width)
      (setf (aref mom (+ y dy) (+ x dx)) (aref sun dy dx))))
  mom)

(defun matrix-append
  (m-list
   &optional (dir '(1 1)))
  (let* ((row (reduce #'(lambda (a b) (+ a (* (car dir) b)))
                      (mapcar #'(lambda (m) (m . dim0)) m-list)))
         (col (reduce #'(lambda (a b) (+ a (* (cadr dir) b)))
                      (mapcar #'(lambda (m) (m . dim1)) m-list)))
         (ret (make-matrix row col))
         (row-i 0) (col-i 0))
    (mapcar
     #'(lambda (m)
         (matrix-set ret m col-i row-i)
         (setq row-i (+ row-i (* (car dir) (m . dim0))))
         (setq col-i (+ col-i (* (cadr dir) (m . dim1)))))
     m-list)
    ret))

(defun fullbody-ik-test
  (&key
   (init
    (progn
      (cond
       ((not (and (boundp '*robot*) *robot*))
	(setq *robot* (hrp2-simple-detail))))
      (cond
       ((not (and (boundp '*pickview*) *pickview*))
	(pickview :no-menu t)
	(objects *robot*)))))
   (target-limb (list :rarm :larm :rleg :lleg))
   (move-target (mapcar #'(lambda (k) (send *robot* k :end-coords)) target-limb))
   (link-list
    (mapcar
     #'(lambda (k c)
	 (send *robot* :link-list (send c :parent)))
	       ;(if (find-method *robot* k) (send *robot* k :root-link))))
     target-limb
     move-target))
   (all-links (reduce #'union (cons nil link-list)))
   (weight-list (make-list (length move-target) :initial-element (unit-matrix 6)))
   (eus? nil) (reset? t) (move-base? t) (alg SLSQP)
   (x0 (progn
	 (cond
	  (reset?
	   (send *robot* :reset-pose)
	   (send *robot* :newcoords (make-coords :pos (float-vector 0 0 700)))
	   (send *pickview* :draw-objects)))
	 (vector-eus2nlopt (send-all (send-all all-links :joint) :joint-angle))))
   (target-coords ;*target-coords*)
    (mapcar #'(lambda (k c)
		(cond
		 ((find k '(:rleg :lleg)) (copy-object (send c :worldcoords)))
		 (t
		  (make-coords :pos (v+ (float-vector -900 (aref (send c :worldpos) 1) 0)
					(random-vector 0.0))
			       :rpy (map cons #'+
					 (random-vector 0.0))))))
	    target-limb move-target))
   (debug-view :no-message)
   (add-6dof? t)
   ret
   )
  (cond
   (eus?
    (send *robot* :fullbody-inverse-kinematics
	  target-coords
	  :target-centroid-pos nil
	  :debug-view debug-view
	  :stop 100
	  :link-list link-list
	  :revert-if-fail nil
	  :move-target move-target))
   (t (send
       (instance
	nlopt-ik
	:init
	:add-6dof? add-6dof?
	:debug-view debug-view
	:robot *robot*
	:move-target move-target
	:target-coords target-coords
	:link-list link-list
	:all-links all-links
	:weight-list weight-list
	;;
	:algorithm alg
	:x0 x0
	:x-min
	(vector-eus2nlopt (send-all (send-all all-links :joint) :min-angle))
	:x-max
	(vector-eus2nlopt (send-all (send-all all-links :joint) :max-angle))
	:ftol 1e-6 :xtol 1e-6 :eqthre 1e-2
	:m-x (length x0)
	:m-g 0
	:m-h 0)
       :optimize)))
  )

(defclass nlopt-ik
  :super nlopt-object
  :slots (robot
	  move-target
	  target-coords
	  link-list
	  all-links
	  weight-list
	  ;;
	  add-6dof?
	  debug-view
	  ))

(defmethod nlopt-ik
  (:init
   (&rest
    args
    &key
    (robot *robot*)
    move-target
    (target-coords
     (mapcar
      #'(lambda (mt)
	  (copy-object (send mt :worldcoords)))
      move-target))
    (link-list
     (mapcar
      #'(lambda (c) (send robot :link-list (send c :parent)))
      move-target))
    (all-links (reduce #'union (cons nil link-list)))
    (weight-list
     (make-list (length move-target)
		:initial-element (unit-matrix 6)))
    ;;
    (stop -1)
    (add-6dof? t)
    (debug-view :no-message)
    &allow-other-keys
    )
   (eval
    (send-super*
     :init
     :x0
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :joint-angle))
     :x-min
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :min-angle))
     :x-max
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :max-angle))
     :ftol 1e-6 :xtol 1e-6 :eqthre 1e-2
     :m-x
     (length
      (vector-eus2nlopt
       (send-all (send-all all-links :joint) :joint-angle)))
     :m-g 0
     :m-h 0
     :max-eval stop
     args))
   (mapcar
    #'(lambda (k val) (send self k val))
    (list :robot :move-target :target-coords
	  :link-list :all-links :weight-list
	  :add-6dof? :debug-view)
    (list robot move-target target-coords
	  link-list all-links weight-list
	  add-6dof? debug-view))
   )
  (:f (v1 v2)
      (let (buf (tmp 0))
	(mapcar
	 #'(lambda (j v) (send j :joint-angle v))
	 (send-all all-links :joint)
	 (vector-nlopt2eus v1 :link-list all-links))
	(cond
	 ((and debug-view
	       (boundp '*viewer*)
	       *viewer*)
	  (send *viewer* :draw-objects)
	  (x::window-main-one)))
	(setf (aref v2 0)
	      (apply
	       #'+
	       (mapcar
		#'(lambda (w m c)
		    (setq buf (concatenate
			       float-vector
			       (scale 1e-3 (send m :difference-position c))
			       (send m :difference-rotation c)))
		    (norm2 (transform w buf)))
		weight-list move-target target-coords))))
      0)
  (:df (v1 v2)
       (let (dx J buf)
	 (mapcar
	  #'(lambda (j v) (send j :joint-angle v))
	  (send-all all-links :joint)
	  (vector-nlopt2eus v1 :link-list all-links))
	 (cond
	  ((and debug-view
		(boundp '*viewer*)
		*viewer*)
	   (send *viewer* :draw-objects)
	   (x::window-main-one)))
	 (my-copy-matrix
	  (reduce
	   #'v+
	   (append
	    (list (scale 0 v2) (scale 0 v2))
	    (mapcar
	     #'(lambda (w l m c)
		 (setq J (send robot :calc-jacobian-from-link-list l
			       :move-target m
			       :target-coords (make-coords)
			       :translation-axis '(t)
			       :rotation-axis '(t)))
		 (setq dx
		       (transform
			w
			(concatenate
			 float-vector
			 (scale 1e-3 (send m :difference-position c))
			 (send m :difference-rotation c))))
		 (setq buf
		       (mapcar
			#'cons
			l
			(vector-nlopt2eus
			 (transform (transpose J) (scale -2.0 dx))
			 :link-list l)))
		 (vector-eus2nlopt
		  (mapcar
		   #'(lambda (al)
		       (let* ((val (cdr (assoc al buf)))
			      (val0 (send (send al :joint) :joint-angle)))
			 (cond
			  ((and val (vectorp val)) (scale 1e-3 val))
			  (val val)
			  ((vectorp val0) (scale 0 val0))
			  (t 0))))
		   all-links)))
	     weight-list link-list move-target target-coords)))
	  v2))
       0)
  (:g (v1 v2) 0)
  (:dg (v1 v2) 0)
  (:h (v1 v2) 0)
  (:dh (v1 v2) 0)
  (:eus-ik
   nil
   (send robot :fullbody-inverse-kinematics
	 target-coords
	 :target-centroid-pos nil
	 :debug-view debug-view
	 :stop 100
	 :link-list link-list
	 :revert-if-fail nil
	 :move-target move-target))
  (:optimize
   (&rest args)
   (cond
    (add-6dof?
      (let ((ret)
	    (ll-buf link-list)
	    (al-buf all-links)
	    (add6dof (add-6dof-joint :link-list link-list)))
	(send self :link-list
	      (cdr (assoc :6dof-links add6dof)))
	(send self :all-links (reduce #'union (cons nil link-list)))
	(setq ret
	      (send-super*
	       :optimize
	       :x0
	       (vector-eus2nlopt
		(send-all (send-all all-links :joint) :joint-angle))
	       :x-min
	       (vector-eus2nlopt
		(send-all (send-all all-links :joint) :min-angle))
	       :x-max
	       (vector-eus2nlopt
		(send-all (send-all all-links :joint) :max-angle))
	       :m-x
	       (length
		(vector-eus2nlopt
		 (send-all (send-all all-links :joint) :joint-angle)))
	       args))
	(send self :link-list ll-buf)
	(send self :all-links al-buf)
	(funcall (cdr (assoc :del-6dof-links add6dof)))
	ret))
     (t (send-super* :optimize args))))
  )
