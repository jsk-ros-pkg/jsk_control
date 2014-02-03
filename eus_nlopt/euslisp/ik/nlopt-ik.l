;#-:jsk (jsk)
;#-:rbrain-basic (rbrain)

(require "../nlopt-object.l")

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
       (make-list 6 :initial-element 1));;1e-3))
      (scala-scale 1));;(deg2rad 1)))
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
      (make-list 6 :initial-element 1));;1e-3))
     (scala-scale 1);;(rad2deg 1))
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

(defvar *best-cost*)
(defvar *best-pos*)
(defun fullbody-ik-test
  (&rest
   args
   &key
   (viewer
    (progn
      (cond
       ((not (and (boundp '*robot*) *robot*))
	(require "euslib/rbrain/irtrobot/hrp4r.l")
	(setq *robot* (hrp4r))))
      (cond
       ((and (not (and (boundp '*pickview*) *pickview*))
	     (boundp 'pickview))
	(pickview :no-menu t)
	(objects *robot*)
	*pickview*)
       ((not (and (boundp '*pickview*) *pickview*))
	(objects *robot*)
	*irtviewer*))))
   (target-limb (list :rarm :larm :rleg :lleg))
   (move-target (mapcar #'(lambda (k) (send *robot* k :end-coords)) target-limb))
   (link-list
    (mapcar
     #'(lambda (c)
	 (send *robot* :link-list (send c :parent)))
	       ;(if (find-method *robot* k) (send *robot* k :root-link))))
     move-target))
   (all-links (reduce #'union (cons nil link-list)))
   (axis-matrix (make-list (length move-target) :initial-element (unit-matrix 6)))
   (reset? t) (move-base? t) (alg 11)
   (initial-state (progn
	 (cond
	  (reset?
	   (send *robot* :reset-pose)
	   (send *robot* :newcoords (make-coords :pos (float-vector 0 0 700)))
	   (if viewer (send viewer :draw-objects))))
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
   (max-time 20)
   (stop -1) (max-eval stop)
   (xtol 1e-6) (ftol 1e-6)
   (debug-view :success-draw)
   (optimize :optimize)
   (add-6dof? t)
   (max (scale 1 #F(1000 1000 1000 200 200 200)))
   (min (scale -1 #F(1000 1000 1000 200 200 200)))
   (f-callback
    '(lambda (v1 v2)
       (cond
	((or (null *best-cost*) (< (aref v2 0) *best-cost*))
	 (setq *best-cost* (aref v2 0))
	 (setq
	  *best-pos*
	  (list
	   (cons :angle-vector (copy-object (send *robot* :angle-vector)))
	   (cons :worldcoords (copy-object (send *robot* :worldcoords)))))))))
   ret
   &allow-other-keys
   )
  (send
   (instance*
    nlopt-ik
    :init
    (append
     args
     (list
      :add-6dof? add-6dof?
      :debug-view debug-view
      :robot *robot*
      :move-target move-target
      :target-coords target-coords
      :link-list link-list
      :all-links all-links
      :axis-matrix axis-matrix
      :max-eval max-eval
      :max-time max-time
      :f-callback f-callback
      :min min :max max
      ;;
      :algorithm alg
      :initial-state initial-state
      :state-min-vector
      (vector-eus2nlopt (send-all (send-all all-links :joint) :min-angle))
      :state-max-vector
      (vector-eus2nlopt (send-all (send-all all-links :joint) :max-angle))
      :ftol ftol :xtol xtol :eqthre 1e-2
      :state-dimension (length initial-state)
      :equality-dimension 0
      :inequality-dimension 0)))
   optimize))

(defun ik-test
  (&rest
   args
   &key
   (viewer
    (progn
      (cond
       ((not (and (boundp '*robot*) *robot*))
	(require "euslib/rbrain/irtrobot/hrp4r.l")
	(setq *robot* (hrp4r))))
      (cond
       ((and (not (and (boundp '*pickview*) *pickview*))
	     (boundp 'pickview))
	(pickview :no-menu t)
	(objects *robot*)
	*pickview*)
       ((not (and (boundp '*pickview*) *pickview*))
	(objects *robot*)
	*irtviewer*))))
   (target-limb :rarm)
   (move-target (send *robot* target-limb :end-coords))
   (link-list
    (send *robot* :link-list (send move-target :parent)))
   (all-links link-list)
   (axis-matrix (unit-matrix 6))
   (reset? t)
   (alg 11)
   (initial-state (progn
	 (cond
	  (reset?
	   (send *robot* :reset-pose)
	   (send *robot* :newcoords (make-coords :pos (float-vector 0 0 700)))
	   (if viewer (send viewer :draw-objects))))
	 (vector-eus2nlopt
	  (send-all (send-all all-links :joint) :joint-angle))))
   (target-coords ;*target-coords*)
    (make-coords :pos (v+ (send move-target :worldpos)
			  #F(500 0 500))))
   (debug-view :success-draw)
   (optimize :optimize)
   (add-6dof? nil)
   (max-time 20)
   (stop -1) (max-eval stop)
   (xtol 1e-6) (ftol 1e-6)
   ret
   &allow-other-keys
   )
  (send
   (instance*
    nlopt-ik
    :init
    (append
     args
     (list
      :add-6dof? add-6dof?
      :debug-view debug-view
      :robot *robot*
      :move-target (list move-target)
      :target-coords (list target-coords)
      :link-list (list link-list)
      :all-links all-links
      :axis-matrix (list axis-matrix)
      :max-eval max-eval
      :root-link-virtual-joint-weight #F(0 0 0 0 0 0)
      ;;
      :algorithm alg
      :initial-state initial-state
      :state-min-vector
      (vector-eus2nlopt (send-all (send-all all-links :joint) :min-angle))
      :state-max-vector
      (vector-eus2nlopt (send-all (send-all all-links :joint) :max-angle))
      :max-time max-time
      :ftol ftol :xtol xtol :eqthre 1e-2
      :state-dimension (length initial-state)
      :equality-dimension 0
      :inequality-dimension 0)))
   optimize))

(defun fill0-vector-with-links
  (&key
   (all-links (cdr (send *robot* :links)))
   (link-list (send *robot* :rarm :links))
   (val-list
    (send-all
     (send-all link-list :joint) :joint-angle))
   buf
   )
  (setq buf (mapcar #'cons link-list val-list))
  (vector-eus2nlopt
   (mapcar
    #'(lambda (al)
	(let* ((val (cdr (assoc al buf)))
	       (val0 (send (send al :joint) :joint-angle)))
	  (cond
	   ((and val (vectorp val)) val)
	    ;;(map float-vector #'*
	    ;;root-link-virtual-joint-weight val))
	   (val val)
	   ((vectorp val0) (scale 0 val0))
	   (t 0))))
    all-links)))

(defconstant OPTIMIZE_EUS_IK NUM_ALGORITHMS)
(defclass nlopt-ik
  :super nlopt-object
  :slots (robot
	  move-target
	  target-coords
	  link-list
	  all-links
	  root-link-virtual-joint-weight
	  weight-list
	  axis-matrix
	  max min
	  collision-avoidance-link-pair
	  ;;
	  translation-axis
	  rotation-axis
	  ;;
	  target-centroid-pos
	  cog-gain
	  ;;
	  add-6dof?
	  debug-view
	  ;;
	  last-cost
	  ;;
	  f-callback
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
    (target-centroid-pos nil)
    (cog-gain 1)
    ;;
    (translation-axis (mapcar #'(lambda (hoge) t) move-target))
    (rotation-axis (mapcar #'(lambda (hoge) t) move-target))
    ;;
    (link-list
     (mapcar
      #'(lambda (c) (send robot :link-list (send c :parent)))
      move-target))
    (all-links
     (if target-centroid-pos (cdr (send robot :links))
       (reduce #'union (cons nil link-list))))
    (axis-matrix
     (make-list (length move-target)
		:initial-element (unit-matrix 6)))
    (additional-weight-list nil)
    (weight-list
     (mapcar
      #'(lambda (l)
	  (or (cadr (assoc l additional-weight-list))
	      1))
      all-links))
    (root-link-virtual-joint-weight
     (coerce (make-list 6 :initial-element 1e-3) float-vector))
    (min (float-vector -1500 -1500  -1500 -200 -200 -200))
    (max (float-vector 1500 1500 1500 200 200 200))
    (collision-avoidance-link-pair nil)
    ;;
    (algorithm L_BFGS)
    ((:initial-state initial-state)
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :joint-angle)))
    ((:state-min-vector state-min-vector)
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :min-angle)))
    ((:state-max-vector state-max-vector)
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :max-angle)))
    (ftol 1e-18)
    (xtol 1e-18)
    (eqthre 1e-10)
    (state-dimension
     (length
      (vector-eus2nlopt
       (send-all (send-all all-links :joint) :joint-angle))))
    (equality-dimension 0)
    (inequality-dimension 0)
    ;;
    (stop -1)
    (max-eval stop)
    (add-6dof? t)
    (debug-view :success-draw)
    (f-callback)
    &allow-other-keys
    )
   (send-super*
    :init
    :algorithm algorithm
    :initial-state initial-state
    :state-min-vector state-min-vector
    :state-max-vector state-max-vector
    :ftol ftol
    :xtol xtol
    :eqthre eqthre
    :state-dimension state-dimension
    :equality-dimension equality-dimension
    :inequality-dimension inequality-dimension
    :max-eval max-eval
    args)
   ;; list check
   (mapcar
    #'(lambda (k val)
	(send self
	      k
	      (cond
	       ((atom val) (list val))
	       (t val))))
    (list :move-target :target-coords
	  :link-list :weight-list :axis-matrix
	  :translation-axis :rotation-axis)
    (list move-target target-coords
	  link-list weight-list axis-matrix
	  translation-axis rotation-axis))
   ;;
   (mapcar
    #'(lambda (k val) (send self k val))
    (list :robot :all-links
	  :add-6dof? :debug-view :root-link-virtual-joint-weight
	  :target-centroid-pos :cog-gain
	  :collision-avoidance-link-pair
	  :min :max :f-callback)
    (list robot all-links
	  add-6dof? debug-view root-link-virtual-joint-weight
	  target-centroid-pos cog-gain
	  collision-avoidance-link-pair
	  min max f-callback))
   )
  (:dump-args
   nil
   (apply
    #'append
    (mapcar
     #'(lambda (kv)
	 (if (cdr kv)
	     (list
	      (read-from-string (format nil ":~A" (car kv)))
	      (cdr kv))))
     (send self :slots))))
  (:target-diff
   (&key
    ((:axis-matrix am) axis-matrix)
    ((:move-target mt) move-target)
    ((:target-coords tc) target-coords)
    ((:translation-axis ta) translation-axis)
    ((:rotation-axis ra) rotation-axis))
   (mapcar
    #'(lambda (w m c ta ra)
	(transform
	 w
	 (scale
	  1e+0
	  (concatenate
	   float-vector
	   (scale 1e-3 (send m :difference-position c
			     :translation-axis ta))
	   (send m :difference-rotation c
		 :rotation-axis ra)))))
    am mt tc ta ra))
  (:Jxdx
   (&key
    ((:link-list ll) link-list)
    ((:move-target mt) move-target)
    J)
   (mapcar
    #'(lambda (l m dx)
	(setq J (send robot :calc-jacobian-from-link-list l
		      :move-target m
		      :target-coords nil
		      :translation-axis '(t)
		      :rotation-axis '(t)))
	(transform (transpose J) dx))
    ll mt (send self :target-diff)))
  (:Jcdc
   (&key
    ((:link-list ll) all-links)
    ((:cog-gain cg) cog-gain)
    ((:translation-axis ta) :z)
    ((:target-centroid-pos tcp) target-centroid-pos)
    (centroid-offset-func)
    &allow-other-keys)
   (transform
    (transpose
     (send robot :calc-cog-jacobian-from-link-list
	   :link-list ll
	   :translation-axis ta))
    (scale cg
	   (calc-dif-with-axis
	    (let ((current-centroid-pos
		   (if (functionp centroid-offset-func)
		       (funcall centroid-offset-func)
		     (send robot :centroid nil))))
	      (scale 1e-3 (v- tcp current-centroid-pos)))
	    ta))
    )
   )
  (:update-viewer
   (&key
    (debug-view debug-view)
    (target-coords target-coords)
    (cost 0))
   (cond
    ((or
      (not debug-view)
      (not (and (boundp '*viewer*)
		*viewer*)) nil))
    ((and (eq debug-view :success-draw)
	  (< cost last-cost))
     (setq last-cost cost)
     (send *viewer* :draw-objects :flush nil)
     (send-all target-coords
	       :draw-on :flush nil :color #F(1 0 0) :size 100)
     (send *viewer* :viewsurface :flush)
     (x::window-main-one)
     )
    ((eq debug-view :success-draw)
     (x::window-main-one)
     )
    (t
     (send *viewer* :draw-objects :flush nil)
     (send-all target-coords
	       :draw-on :flush nil :color #F(1 0 0) :size 100)
     (send *viewer* :viewsurface :flush)
     (x::window-main-one))))
  ;;
  (:evaluation-function
   (v1 v2)
   (mapcar
    #'(lambda (j v) (send j :joint-angle v))
    (send-all all-links :joint)
    (vector-nlopt2eus
     v1 :link-list all-links))
   (setf (aref v2 0)
	 (*
	  1e+3
	  (+
	   (if target-centroid-pos
	       (norm2
		(subseq
		 (scale (* cog-gain 1e-3)
			(v- (send robot :centroid) target-centroid-pos))
		 0 2))
	     0)
	   (apply #'+ (mapcar #'norm2 (send self :target-diff))))))
   ;;
   ;; draw
   ;;
   (if (functionp f-callback)
       (funcall f-callback v1 v2))
   (send self :update-viewer
	 :debug-view debug-view
	 :target-coords target-coords
	 :cost (aref v2 0))
   0)
  (:evaluation-function-gradient
   (v1 v2)
   (mapcar
    #'(lambda (j v) (send j :joint-angle v))
    (send-all all-links :joint)
    (vector-nlopt2eus v1 :link-list all-links))
   (send robot :worldcoords)
   (my-copy-matrix
    (scale
     1e+3
     (map float-vector
	  #'*
	  weight-list
	  (vlist+
	   (mapcar
	    #'(lambda (ll Jdx)
		(fill0-vector-with-links
		 :all-links all-links
		 :link-list ll
		 :val-list
		 (vector-nlopt2eus
		  (scale -2.0 Jdx) :link-list ll)))
	    (append
	     (if target-centroid-pos (list all-links))
	     link-list)
	    (append
	     (if target-centroid-pos (list (send self :Jcdc)))
	     (send self :Jxdx))))))
    v2)
   0)
  (:equality-function (v1 v2) 0)
  (:equality-function-gradient (v1 v2) 0)
  (:inequality-function (v1 v2) 0)
  (:inequality-function-gradient (v1 v2) 0)
  (:eus-ik
   (&rest args)
   (format t "[ALGORITHM]: EUS_IK(inf)~%")
   (bench
    (apply
     #'send
     (append
      (list robot :fullbody-inverse-kinematics)
      (list target-coords)
      (list :revert-if-fail nil)
      (list :stop 100)
      (list :debug-view
	    (cond
	     ((eq debug-view :success-draw) :no-message)
	     (t debug-view)))
      args
      (send self :dump-args)))))
  (:optimize
   (&rest args)
   (setq last-cost 1e+1000)
   (cond
    ((>= algorithm OPTIMIZE_EUS_IK)
     (apply
      #'send
      (append
       (list self :eus-ik)
       args)))
    (add-6dof?
      (let ((ret)
	    (ll-buf link-list)
	    (al-buf all-links)
	    (add6dof (add-6dof-joint :link-list link-list
				     :max max :min min)))
	(send self :link-list
	      (cdr (assoc :6dof-links add6dof)))
	(send self :all-links (reduce #'union (cons nil link-list)))
	(send self :weight-list
	      (concatenate
	       cons
	       root-link-virtual-joint-weight
	       weight-list))
	(setq ret
	      (send-super*
	       :optimize
	       :initial-state
	       (vector-eus2nlopt
		(send-all (send-all all-links :joint) :joint-angle))
	       :state-min-vector
	       (vector-eus2nlopt
		(send-all (send-all all-links :joint) :min-angle))
	       :state-max-vector
	       (vector-eus2nlopt
		(send-all (send-all all-links :joint) :max-angle))
	       :state-dimension
	       (length
		(vector-eus2nlopt
		 (send-all (send-all all-links :joint) :joint-angle)))
	       args))
	(send self :link-list ll-buf)
	(send self :all-links al-buf)
	(funcall (cdr (assoc :del-6dof-links add6dof)))
	(if (and (boundp '*viewer*) *viewer*) (send *viewer* :draw-objects))
	ret))
    (t (send-super* :optimize
		    (append
		     (list :root-link-virtual-joint-weight
			   #F(0 0 0 0 0 0)
			   :min #F(0 0 0 0 0 0)
			   :max #F(0 0 0 0 0 0))
		     args))))
   (list
    (cons :angle-vector (copy-object (send robot :angle-vector)))
    (cons :worldcoords (copy-object (send robot :worldcoords)))
    (cons :dif
	  (mapcar
	   #'(lambda (mt tr)
	       (concatenate
		float-vector
		(scale 1e-3 (send mt :difference-position tr))
		(send mt :difference-rotation tr)))
	   move-target target-coords)))
   )
  )

;; test mode
(cond
 ((boundp '*alg*)
  (let* ((i (- *alg* 1))
	 (s 250)
	 (w (send x::*root* :width))
	 (h (send x::*root* :height))
	 (x (mod (* s i) (- w s)))
	 (y (* s (/ (* s i) (- w s))))
	 timer)
    (pickview :width s :height s :no-menu t)
    (setq *robot* (hrp2-simple-detail))
    (objects (list *robot*))
    (send *pickview* :move x y)
    (send *pickview* :draw-objects)
    (while (not (probe-file "/tmp/hoge"))
      (unix:sleep 1))
    (setq timer
	  (bench2 (ik-test :alg *alg*)))
    (if (< timer 20) (unix:usleep (round (- (* 20 1e+6) (* timer 1e+6)))))
    (exit -1))))

#|

(defun place-graph-in-order
  (&key
   (graph *graph-sample*)
   (size (list (send (car graph) :width)
               (send (car graph) :height)))
   (root-size (list (- (send x::*root* :width) (car size))
                    (- (send x::*root* :height) (cadr size))))
   (x 0)
   (y 0))
  (if (null graph)
      nil
    (let ((g (car graph)))
      (send g :move x y)
      (send g :repaint)
      (setq x (+ x (car size)))
      (if (> x (car root-size))
          (progn
            (setq x 0)
            (setq y (+ y (cadr size)))))
      (if (> y (cadr root-size))
          (setq y 0))
      (place-graph-in-order
       :graph (cdr graph)
       :size size
       :root-size root-size
       :x x
       :y y))))

(dotimes (i 18) (ik-test :alg i))

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
       :debug-view nil
       :move-target
       (mapcar #'(lambda (k) (send *robot* k :end-coords))
	       '(:rleg :lleg))
       :target-centroid-pos
       (v+
	#f(100 0 0)
	(scale
	 0.5
	 (apply
	  #'v+ (list (send *robot* :rleg :end-coords :worldpos)
		     (send *robot* :lleg :end-coords :worldpos)))))))

(send a :optimize)
