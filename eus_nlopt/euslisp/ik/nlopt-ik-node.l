#!/usr/bin/env roseus

(require "nlopt-ik.l")

(defun nlopt-log
  (str &rest args)
  (let* ((message
	  (apply
	   #'format
	   (append (list nil str) args)))
	 (pre-message
	  (format nil "[nlopt-ik] ~A" message)))
    (ros::ros-info pre-message)
    ;;(format t "~A~%" pre-message)
    ))

(defvar *algorithm*
  (let ((str (unix::getenv "NLOPT_IK_NODE_ALGORITHM")))
    (cond
     ((and str (plusp (length str)))
      (read-from-string str))
     (t L_BFGS))))

(defvar *robot*
  (let ((str (unix::getenv "NLOPT_IK_NODE_ROBOT")))
    (nlopt-log "robot type=~A" str)
    (setq
     str
     (cond
      ((and str (plusp (length str)))
       (send (read-from-string str) :pname))
      (t "ATLAS_V3")))
    (cond
     ((string= str "HRP4R")
      (require "euslib/rbrain/irtrobot/hrp4r.l")
      (hrp4r))
     ((string= str "STARO")
      (require "euslib/rbrain/irtrobot/staro.l")
      (staro))
     ((string= str "ATLAS")
      (require "euslib/rbrain/irtrobot/atlas.l")
      (atlas))
     (t ;;(string= str "ATLAS_V3")
      (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
      (atlas-with-hand))
     )))

(defvar *nlopt_ik_node_name*
  (format nil "nlopt_ik_node_~A" (send *robot* :name)))

;;(objects *robot*)

(ros::roseus-add-msgs "std_msgs")
(ros::roseus *nlopt_ik_node_name*)

(defun dump-object
  (obj
   &optional buf)
  (cond
   ((stringp obj)
    (format nil "\"~A\"" obj))
   ((or (vectorp obj)
	(matrixp obj)
	(numberp obj)
	(symbolp obj))
    (format nil "~A" obj))
   ((listp obj)
    (format nil
	    "~A"
	    (append
	     (mapcar #'dump-object obj)
	     (if (cdr (last obj))
		 (dump-object (cdr (last obj)))))))
   (t
    (setq buf
	  (format nil
		  "~A"
		  (apply
		   #'append
		   (mapcar
		    #'(lambda (kv)
			(list (car kv)
			      (dump-object
			       (cdr kv))))
		    (send obj :slots)))))
    (format nil "#s(~A ~A)"
	    (send (class obj) :name)
	    (subseq buf 1 (- (length buf) 1))
	    ))
   ))

(defun check-ik-param
  (str)
  (let* ((kv (read-from-string (format nil "(~A)" str))))
    ;; flatten
    (nlopt-log "raw ~A" kv)
    (while (and kv
		(listp kv)
		(not (keywordp (cadr kv)))
		(not (keywordp (car kv))))
      (setq kv (car kv)))
    (nlopt-log "tar ~A" kv)
    ;;
    (cond
     ((or (null kv) (not (listp kv)))
      (nlopt-log "null args ~A" kv)
      (return-from check-ik-param nil)))
    (cond
     ((not (keywordp (car kv)))
      (setq kv
	    (cons :target-coords kv))))
    ;; eval
    (setq kv
	  (mapcar
	   #'(lambda (kv)
	       (cond
		((functionp kv)
		 (funcall kv))
		((and (listp kv)
		      (functionp (car kv)))
		 (eval kv))
		(t kv)))
	   kv))
    ;;
    (cond
     ((or
       (flatten
	(mapcar
	 #'(lambda (c)
	     (not (subclassp (class c) coordinates)))
	 (car kv)))
       (flatten
	(mapcar
	 #'(lambda (c)
	     (not (subclassp (class c) cascaded-coords)))
	 (cadr (member :move-target kv)))))
      (nlopt-log "invalid args ~A" kv)
      (return-from solve-nlopt-ik nil)))
    (cond
     ((vectorp (cadr (member :angle-vector kv)))
      (send *robot* :angle-vector
	    (cadr (member :angle-vector kv)))))
    (cond
     ((subclassp (class (cadr (member :base-coords kv)))
		 coordinates)
      (send *robot* :newcoords
	    (or
	     (cadr (member :worldcoords kv))
	     (cadr (member :base-coords kv))))))
    kv
    ))

(defvar *kv*)
(defun solve-nlopt-ik-thread
  (msg)
  (let* ((str (send msg :data))
	 (kv (check-ik-param str))
	 ret)
    ;; (setq ret
    ;; 	  (apply #'fullbody-ik-test
    ;; 		 (append
    ;; 		  kv
    ;; 		  (list :init nil)
    ;; 		  (list :viewer nil)
    ;; 		  (list :reset? nil)
    ;; 		  (list :algorithm *algorithm*))))
    ;; ;;
    ;; (ros::publish "/nlopt_ik_node/ik_response"
    ;; 		  (instance std_msgs::string
    ;; 			    :init
    ;; 			    :data
    ;; 			    (dump-object ret)))
    (setq *kv* kv)
    (sys::make-thread 1)
    (sys::thread
     #'(lambda nil
	 (apply #'fullbody-ik-test
		(append
		 *kv*
		 (list :init nil)
		 (list :viewer nil)
		 (list :reset? nil)
		 (list :algorithm *algorithm*)))
	 (setq *best-pos* nil)
	 ))
    ))

(defun solve-nlopt-ik
  (msg)
  (let* ((str (send msg :data))
	 (kv (check-ik-param str))
	 ret)
    (setq ret
    	  (apply #'fullbody-ik-test
    		 (append
    		  kv
    		  (list :init nil)
    		  (list :viewer nil)
    		  (list :reset? nil)
    		  (list :algorithm *algorithm*))))
    ;;
    (ros::publish "/nlopt_ik_node/ik_response"
    		  (instance std_msgs::string
    			    :init
    			    :data
    			    (dump-object ret)))
    ret
    ))

(cond
 (*algorithm*
  (ros::subscribe "/nlopt_ik_node/ik_request"
		  std_msgs::string
		  #'solve-nlopt-ik 1)
  (ros::advertise "/nlopt_ik_node/ik_response"
		  std_msgs::string)
  ;;
  (ros::rate 3)
  (do-until-key
   (if (not (ros::ok))
       (return-from nil nil))
   (ros::spin-once)
   ;; (ros::publish "/nlopt_ik_node/ik_response"
   ;; 		 (instance std_msgs::string
   ;; 			   :init
   ;; 			   :data
   ;; 			   (dump-object *best-pos*)))
   (ros::sleep))))
