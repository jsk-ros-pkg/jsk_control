#!/usr/bin/env roseus

;; (defvar *learn-mode* :rarm)
(require "learn.l")

(defun gen-viewer
  (title)
  (let* ((irtviewer (make-irtviewer)))
    (send irtviewer :title title)
    (send irtviewer :change-background (float-vector 1 1 1))
    (send irtviewer :resize 300 300)
    (send irtviewer :set-val 'x::draw-origin nil)
    (send irtviewer :set-val 'x::draw-floor nil)
    (objects (append (if (eq *learn-mode* :rarm-larm-rleg-lleg) (list (car (send *robot* :links))) nil)
		     *arrow* (send-all *joint-list1* :child-link)))
    ;; (send irtviewer :viewer :viewing :look #f(2531.18 1116.03 2930.59) #f(260.394 -91.3679 1195.88))
    (send irtviewer :viewer :draw-objects)
    irtviewer))

(defun draw-mview
  nil
  (mapcar
   '(lambda (ar c) (send ar :newcoords (send c :copy-worldcoords)))
   *arrow*
   (send-all *ef-coords* :worldcoords))
  (send-all (list *mview1* *mview2*) :draw-objects))

(defun draw-util
  (irtviewer redundancy &optional (viewer (send irtviewer :viewer)))
  (if (not (eq (send irtviewer :width) 300)) (send viewer :resize 300 300))
  (send viewer :draw-objects :flush nil)
  (send viewer :viewsurface :color (float-vector 0 0 0))
  (send viewer :viewsurface :string 10 30 (format nil "i: ~A" (map cons '(lambda (v) (* 0.1 (round (* v 10)))) redundancy)))
  (send viewer :viewsurface :flush))

(defvar *mview1* (gen-viewer "w/o ik"))
(unix::sleep 1)
(defvar *mview2* (gen-viewer "w/  ik"))
(unix::sleep 1)

;;

(smart-initialize-net)

(defun _random-angle
  (&optional (joint-list *joint-list1*)
	     (root-link-fix-coords-func *root-link-fix-coords-func*))
  (let* ((ret
	  (mapcar '(lambda (j) (send j :joint-angle
				     (-
				      (+ (send j :min-angle)
					 (* (random 1.0)
					    (- (send j :max-angle)
					       (send j :min-angle))))
				      (send j :joint-angle))
				     :relative t))
		  joint-list)))
    (if (functionp root-link-fix-coords-func)
	(funcall root-link-fix-coords-func))
    (send-all (send-all joint-list :child-link) :worldcoords)
    ret))

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

(defun ik-random-test-loop
  (&rest
   args
   &key
   (joint-list *joint-list1*)
   (ef-coords *ef-coords*)
   (root-link *root-link*)
   (root-link-fix-coords-func *root-link-fix-coords-func*)
   (random-angle (_random-angle joint-list root-link-fix-coords-func))
   (update-coords (send-all ef-coords :worldcoords))
   (c (send-all (send-all ef-coords :worldcoords) :copy-worldcoords))
   (b (send root-link :copy-worldcoords))
   (av (instantiate float-vector (length joint-list)))
   (input (get-vector-from-ef-coords :ef-coords ef-coords :root-link root-link))
   (redundancy (instantiate float-vector (- (caffe::get-blob-count "input") (length input))))
   (step-max 500)
   (redundancy-step
    (/ 1.0 (expt step-max (/ 1.0 (length redundancy)))))
   (redundancy-step-cnt
    (max 1 (floor (/ 1.0 redundancy-step))))
   (revert? nil) (draw? t)
   (update-arrow-coords? t)
   (ik? t)
   &allow-other-keys
   )
  (cpplog::change-output-stream "/dev/null")
  (labels ((itter
            (depth &optional (depth0 depth) (prev 0))
            (cond
             ((and (zerop depth) (eq prev 0))
              (let* ((jmax (max 0 (- step-max (expt redundancy-step-cnt depth0)))))
                (dotimes (j jmax)
                  (setf (aref redundancy (random depth0)) (random 1.0))
                  (itter depth depth0 -1))))
             ((zerop depth)
	      (apply 'ik-random-test
		     :random-angle random-angle :update-coords update-coords
		     :c c :b b :av av :input input :redundancy redundancy
		     :update-arrow-coords? update-arrow-coords?
		     :initialize? nil :revert? revert? :draw? nil args)
	      (if update-arrow-coords? (setq update-arrow-coords? nil))
	      (draw-util *mview1* redundancy)
	      (cond
	       ((or (not ik?)
		    (let* ((target-coords (send-all c :copy-worldcoords))
			   (move-target (send-all ef-coords :get-val 'coords)))
		      (send *robot* :inverse-kinematics
			    target-coords
			    :move-target move-target
			    :link-list (mapcar '(lambda (mt) (send *robot* :link-list (send mt :parent))) move-target)
			    :stop 50
			    :thre (make-list (length target-coords) :initial-element 10)
			    :rthre (make-list (length target-coords) :initial-element (deg2rad 8))
			    :warnp nil
			    ;; :target-centroid-pos nil
			    ;; :root-link-virtual-joint-weight (fill (instantiate float-vector 6) 1)
			    ;; :min (fill (instantiate float-vector 6) -400)
			    ;; :max (fill (instantiate float-vector 6) +400)
			    :revert-if-fail t)))
		(draw-util *mview2* redundancy))))
	     (t
	      (dotimes (j redundancy-step-cnt)
                (setf (aref redundancy (- depth 1)) (* j redundancy-step))
                (itter (- depth 1) depth0 (+ prev (- redundancy-step-cnt j 1))))))))
    (itter (length redundancy)))
  ;; (send *viewer* :draw-objects)
  (cpplog::change-output-stream "")
  )

;; (format t "~A ~A~%" (send *mview2* :viewer :viewing :viewpoint) (send *mview2* :viewtarget))
(defun demo-redundancy1
  nil
  (send-all (list  *mview1* *mview2*) :viewer :viewing :look #f(2531.18 1116.03 2930.59) #f(260.394 -91.3679 1195.88))
  (send-all (list  *mview1* *mview2*) :draw-objects)
  (if (not (y-or-n-p)) (return-from demo-redundancy1 nil))
  (send *robot* :reset-manip-pose)
  (send *robot* :rarm :move-end-pos (float-vector 300 0 0) :world)
  (send *viewer* :draw-objects)
  (ik-random-test-loop :random-angle (send-all *joint-list1* :joint-angle))
  ;;
  (if (not (y-or-n-p)) (return-from demo-redundancy1 nil))
  (send *robot* :reset-pose)
  (send *robot* :rarm :move-end-pos (float-vector 200 0 200) :world :rotation-axis nil)
  (send *viewer* :draw-objects)
  (ik-random-test-loop :random-angle (send-all *joint-list1* :joint-angle))
  ;;
  (if (not (y-or-n-p)) (return-from demo-redundancy1 nil))
  (send *robot* :reset-manip-pose)
  (send *robot* :rarm :move-end-pos (float-vector 200 0 0) :world)
  (send *robot* :rarm :shoulder-p :joint-angle -70 :relative t)
  (send *viewer* :draw-objects)
  (ik-random-test-loop :random-angle (send-all *joint-list1* :joint-angle))
  ;;
  )

(defun demo-redundancy2
  nil
  (send-all (list  *mview1* *mview2*) :viewer :viewing :look #f(2531.18 1116.03 2930.59) #f(-10.5957 0.45322 671.917))
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (draw-mview)
  (if (not (y-or-n-p)) (return-from demo-redundancy2 nil))
  (ik-random-test-loop :random-angle (send-all *joint-list1* :joint-angle))
  ;;
  (send *robot* :reset-manip-pose)
  (send *robot* :legs :crotch-p :joint-angle -70 :relative t)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (draw-mview)
  (if (not (y-or-n-p)) (return-from demo-redundancy2 nil))
  (ik-random-test-loop :random-angle (send-all *joint-list1* :joint-angle))
  ;;
  (send *robot* :reset-pose)
  (send *robot* :larm :shoulder-p :joint-angle 40 :relative t)
  (send *robot* :larm :shoulder-r :joint-angle 10 :relative t)
  (send *robot* :rarm :elbow-p :joint-angle -30 :relative t)
  (send *robot* :rarm :shoulder-p :joint-angle -40 :relative t)
  (send *robot* :lleg :crotch-p :joint-angle 20 :relative t)
  (send *robot* :lleg :knee-p :joint-angle 40 :relative t)
  (send *robot* :lleg :crotch-r :joint-angle 30 :relative t)
  (send *robot* :rleg :move-end-pos (float-vector 0 0 100) :world)
  (send *robot* :fix-leg-to-coords (make-coords) :rleg)
  (draw-mview)
  (if (not (y-or-n-p)) (return-from demo-redundancy2 nil))
  (ik-random-test-loop :random-angle (send-all *joint-list1* :joint-angle))
  ;;
  )
