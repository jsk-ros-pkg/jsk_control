;#-:jsk (jsk)
;#-:rbrain-basic (rbrain)

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
   (reset? t) (move-base? t) (alg 11)
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
   (max-time 20)
   (stop -1) (max-eval stop)
   (xtol 1e-6) (ftol 1e-6)
   (debug-view :no-message)
   (optimize :optimize)
   (add-6dof? t)
   ret
   )
  (send
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
    :max-eval max-eval
    :max-time max-time
    ;;
    :algorithm alg
    :x0 x0
    :x-min
    (vector-eus2nlopt (send-all (send-all all-links :joint) :min-angle))
    :x-max
    (vector-eus2nlopt (send-all (send-all all-links :joint) :max-angle))
    :ftol ftol :xtol xtol :eqthre 1e-2
    :m-x (length x0)
    :m-g 0
    :m-h 0)
   optimize))

(defun ik-test
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
   (target-limb :rarm)
   (move-target (send *robot* target-limb :end-coords))
   (link-list
    (send *robot* :link-list (send move-target :parent)))
   (all-links link-list)
   (weight-list (unit-matrix 6))
   (reset? t)
   (alg 11)
   (x0 (progn
	 (cond
	  (reset?
	   (send *robot* :reset-pose)
	   (send *robot* :newcoords (make-coords :pos (float-vector 0 0 700)))
	   (send *pickview* :draw-objects)))
	 (vector-eus2nlopt
	  (send-all (send-all all-links :joint) :joint-angle))))
   (target-coords ;*target-coords*)
    (make-coords :pos (v+ (send move-target :worldpos)
			  #F(500 0 500))))
   (debug-view :no-message)
   (optimize :optimize)
   (add-6dof? nil)
   (max-time 20)
   (stop -1) (max-eval stop)
   (xtol 1e-6) (ftol 1e-6)
   ret
   )
  (send
   (instance
    nlopt-ik
    :init
    :add-6dof? add-6dof?
    :debug-view debug-view
    :robot *robot*
    :move-target (list move-target)
    :target-coords (list target-coords)
    :link-list (list link-list)
    :all-links all-links
    :weight-list (list weight-list)
    :max-eval max-eval
    :root-link-virtual-joint-weight #F(0 0 0 0 0 0)
    ;;
    :algorithm alg
    :x0 x0
    :x-min
    (vector-eus2nlopt (send-all (send-all all-links :joint) :min-angle))
    :x-max
    (vector-eus2nlopt (send-all (send-all all-links :joint) :max-angle))
    :max-time max-time
    :ftol ftol :xtol xtol :eqthre 1e-2
    :m-x (length x0)
    :m-g 0
    :m-h 0)
   optimize))

(defclass nlopt-ik
  :super nlopt-object
  :slots (robot
	  move-target
	  target-coords
	  link-list
	  all-links
	  weight-list
	  root-link-virtual-joint-weight
	  max min
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
    (weight-list
     (make-list (length move-target)
		:initial-element (unit-matrix 6)))
    (root-link-virtual-joint-weight
     (coerce (make-list 6 :initial-element 1e-3) float-vector))
    (min (float-vector -1500 -1500  -1500 -200 -200 -200))
    (max (float-vector 1500 1500 1500 200 200 200))
    ;;
    (algorithm 11)
    (x0
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :joint-angle)))
    (x-min
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :min-angle)))
    (x-max
     (vector-eus2nlopt
      (send-all (send-all all-links :joint) :max-angle)))
    (ftol 1e-18)
    (xtol 1e-18)
    (eqthre 1e-10)
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
	  :translation-axis :rotation-axis
	  :target-centroid-pos :cog-gain
	  :min :max)
    (list robot move-target target-coords
	  link-list all-links weight-list
	  add-6dof? debug-view root-link-virtual-joint-weight
	  translation-axis rotation-axis
	  target-centroid-pos cog-gain
	  min max))
   )
  (:target-diff
   (&key
    ((:weight-list wl) weight-list)
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
    wl mt tc ta ra))
  (:target-jacobian
   (&key
    ((:link-list ll) link-list)
    ((:move-target mt) move-target)
    J)
   (mapcar
    #'(lambda (l m dx)
	(setq J (send robot :calc-jacobian-from-link-list l
		      :move-target m
		      :target-coords nil;(make-coords)
		      :translation-axis '(t)
		      :rotation-axis '(t)))
	(transform (transpose J) dx))
    ll mt (send self :target-diff)))
  (:cog-jacobian
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
  ;;
  (:f (v1 v2)
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
      (cond
       ((or
	 (not debug-view)
	 (not (and (boundp '*viewer*)
		   *viewer*)) nil))
       ((and (eq debug-view :success-draw)
	     (< (aref v2 0) last-cost))
	(setq last-cost (aref v2 0))
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
	(send *viewer* :draw-objects)
	(x::window-main-one)))
      0)
  (:df (v1 v2)
       (let (buf)
	 (mapcar
	  #'(lambda (j v) (send j :joint-angle v))
	  (send-all all-links :joint)
	  (vector-nlopt2eus v1 :link-list all-links))
	 (send robot :worldcoords)
	 (my-copy-matrix
	  (scale
	   1e+3
	   (vlist+
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
	     (append
	      (if target-centroid-pos
		  (list all-links))
	      link-list)
	     (append
	      (if target-centroid-pos
		  (list (send self :cog-jacobian)))
	      (send self :target-jacobian)))))
	  v2))
					;(print v2)
       0)
  (:g (v1 v2) 0)
  (:dg (v1 v2) 0)
  (:h (v1 v2) 0)
  (:dh (v1 v2) 0)
  (:eus-ik
   (&rest args)
   (apply
    #'send
    (append
     (list robot :fullbody-inverse-kinematics
	   target-coords
	   :target-centroid-pos target-centroid-pos
	   :cog-gain cog-gain
	   :root-link-virtual-joint-weight
	   root-link-virtual-joint-weight
	   :translation-axis translation-axis
	   :rotation-axis rotation-axis
	   :debug-view debug-view
	   :stop max-eval
	   :link-list link-list
	   :revert-if-fail nil
	   ;:revert-if-fail t
	   :move-target move-target)
     args)))
  (:optimize
   (&rest args)
   (setq last-cost 1e+1000)
   (cond
    (add-6dof?
      (let ((ret)
	    (ll-buf link-list)
	    (al-buf all-links)
	    (add6dof (add-6dof-joint :link-list link-list
				     :max max :min min)))
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
