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
      (vector-scale
       (make-list 6 :initial-element 1e-3))
      (scala-scale (deg2rad 1)))
  (apply
   #'concatenate
   (cons
    float-vector
    (mapcar
     #'(lambda (v)
	 (cond ((vectorp v)
		(map float-vector #'* vector-scale v))
	       (t (list (* scala-scale v)))))
     vl))))

(defun vector-nlopt2eus
  (v &key
     (link-list *all-links*)
     (joint-list (send-all link-list :joint))
     (vector-scale
      (make-list 6 :initial-element 1e-3))
     (scala-scale (rad2deg 1))
     (index 0) buf ret)
  (mapcar
   #'(lambda (j)
       (setq buf (send j :joint-angle))
       (cond
	((vectorp buf)
	 (map float-vector
	      #'/
	      (subseq v index (setq index (+ index (length buf))))
	      vector-scale))
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

(defun vlist+
  (vlist
   &optional (ret (scale 0 (car vlist))))
  (dolist (v vlist)
    (dotimes (i (length v))
      (setf (aref ret i)
	    (+ (aref ret i) (aref v i)))))
  ret)

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
   (eus? nil) (reset? t) (move-base? t) (alg 8)
   (x0 (progn
	 (cond
	  (reset?
	   (send *robot* :reset-pose)
	   (send *robot* :newcoords (make-coords :pos (float-vector 0 0 700)))
	   (send *pickview* :draw-objects)))
	 (vector-eus2nlopt
	  (send-all (send-all all-links :joint) :joint-angle))))
   (target-coords ;*target-coords*)
    (mapcar #'(lambda (k c)
		(cond
		 ((find k '(:rleg :lleg)) (copy-object (send c :worldcoords)))
		 (t
		  (make-coords :pos (v+ (float-vector -700 (aref (send c :worldpos) 1) 0)
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
	  root-link-virtual-joint-weight
	  ;;
	  target-centroid-pos
	  cog-gain
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
    (root-link-virtual-joint-weight
     (make-list 6 :initial-element 1e-3))
    (target-centroid-pos nil)
    (cog-gain 1)
    ;;
    (algorithm 9)
    (x0
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :joint-angle)))
    (x-min
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :min-angle)))
    (x-max
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :max-angle)))
    (ftol 1e-8)
    (xtol 1e-8)
    (eqthre 1e-4)
    (m-x
     (length
      (vector-eus2nlopt
       (send-all (send-all all-links :joint) :joint-angle))))
    (m-g 0)
    (m-h 0)
    ;;
    (stop -1)
    (max-eval stop)
    (add-6dof? t)
    (debug-view :no-message)
    &allow-other-keys
    )
   (send-super*
    :init
    :algorithm algorithm
    :x0 x0
    :x-min x-min
    :x-max x-max
    :ftol ftol
    :xtol xtol
    :eqthre eqthre
    :m-x m-x
    :m-g m-g
    :m-h m-h
    :max-eval max-eval
    args)
   (mapcar
    #'(lambda (k val) (send self k val))
    (list :robot :move-target :target-coords
	  :link-list :all-links :weight-list
	  :add-6dof? :debug-view :root-link-virtual-joint-weight
	  :target-centroid-pos :cog-gain)
    (list robot move-target target-coords
	  link-list all-links weight-list
	  add-6dof? debug-view root-link-virtual-joint-weight
	  target-centroid-pos cog-gain))
   )
  (:target-diff
   (&key
    (weight-list weight-list)
    (move-target move-target)
    (target-coords target-coords))
   (mapcar
    #'(lambda (w m c)
	(transform
	 w
	 (concatenate
	  float-vector
	  (scale 1e-3 (send m :difference-position c))
	  (send m :difference-rotation c))))
    weight-list move-target target-coords))
  (:target-jacobian
   (&key
    (link-list link-list)
    (move-target move-target)
    J)
   (mapcar
    #'(lambda (l m dx)
	(setq J (send robot :calc-jacobian-from-link-list l
		      :move-target m
		      :target-coords nil;(make-coords)
		      :translation-axis '(t)
		      :rotation-axis '(t)))
	(transform (transpose J) dx))
    link-list move-target
    (send self :target-diff)))
  (:cog-jacobian
   (&key
    (link-list all-links)
    (cog-gain cog-gain)
    (translation-axis :z)
    (target-centroid-pos target-centroid-pos)
    (centroid-offset-func)
    &allow-other-keys)
   (transform
    (transpose
     (send robot :calc-cog-jacobian-from-link-list
	   :link-list link-list
	   :translation-axis translation-axis))
    (scale cog-gain
	   (calc-dif-with-axis
	    (let ((current-centroid-pos
		   (if (functionp centroid-offset-func)
		       (funcall centroid-offset-func)
		     (send robot :centroid nil))))
	      (v- target-centroid-pos current-centroid-pos))
	    translation-axis))
    )
   )
  ;;
  (:f (v1 v2)
      (let (buf (tmp 0))
	(mapcar
	 #'(lambda (j v) (send j :joint-angle v))
	 (send-all all-links :joint)
	 (vector-nlopt2eus
	  v1 :link-list all-links))
	(cond
	 ((and debug-view
	       (boundp '*viewer*)
	       *viewer*)
	  (send *viewer* :draw-objects)
	  (x::window-main-one)))
	(setf (aref v2 0)
	      (+
	       (if target-centroid-pos
		   (norm2
		    (subseq (scale 1e-3 (v- (send robot :centroid) target-centroid-pos))
			    0 2)) 0)
	       (apply #'+ (mapcar #'norm2 (send self :target-diff))))))
      0)
  (:df (v1 v2)
       (let (buf)
	 (mapcar
	  #'(lambda (j v) (send j :joint-angle v))
	  (send-all all-links :joint)
	  (vector-nlopt2eus
	   v1 :link-list all-links))
	 (send robot :worldcoords)
	 (my-copy-matrix
	   (vlist+
	    (append
	     (if target-centroid-pos
		 (scale -2 (send self :cog-jacobian)))
	     (mapcar
	      #'(lambda (l Jdx)
		  (setq buf
			(mapcar
			 #'cons
			 l
			 (vector-nlopt2eus
			  (scale -2.0 Jdx)
			  :link-list l)))
		  (vector-eus2nlopt
		   (mapcar
		    #'(lambda (al)
			(let* ((val (cdr (assoc al buf)))
			       (val0 (send (send al :joint) :joint-angle)))
			  (cond
			   ((and val (vectorp val))
			    (map float-vector #'*
				 root-link-virtual-joint-weight val))
			   (val val)
			   ((vectorp val0) (scale 0 val0))
			   (t 0))))
		    all-links)))
	     link-list
	     (send self :target-jacobian))))
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
	(if (and (boundp '*viewer*) *viewer*) (send *viewer* :draw-objects))
	ret))
    (t (send-super* :optimize args))))
  )


#|
(progn
      (cond
       ((not (and (boundp '*robot*) *robot*))
	(setq *robot* (hrp2-simple-detail))))
      (cond
       ((not (and (boundp '*pickview*) *pickview*))
	(pickview :no-menu t)
	(objects *robot*))))
(setq a
      (instance
       nlopt-ik
       :init
       :move-target
       (mapcar #'(lambda (k) (send *robot* k :end-coords))
	       '(:rarm :larm :rleg :lleg))
       :target-centroid-pos
       (scale
	0.5
	(apply
	 #'v+ (list (send *robot* :rleg :end-coords :worldpos)
		    (send *robot* :lleg :end-coords :worldpos))))))
(send a :optimize)
