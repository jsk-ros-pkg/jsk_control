
(require "nlopt-ik.l")

(defvar *algorithm*
  (let ((str (unix::getenv "NLOPT_IK_NODE_ALGORITHM")))
    (cond
     ((and str (plusp (length str)))
      (read-from-string str))
     (t L_BFGS))))

(defvar *robot*
  (let ((str (unix::getenv "NLOPT_IK_NODE_ROBOT")))
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
     ((string= str "ATLAS_V3")
      (require "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
      (atlas-with-hand))
     ((string= str "ATLAS")
      (require "euslib/rbrain/irtrobot/atlas.l")
      (atlas)))))

(ros::roseus-add-msgs "std_msgs")
(ros::roseus (format nil "nlopt-ik-node-~A" (send *robot* :name)))

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
	    (subseq buf 1 (- (length buf) 2))
	    ))
   ))

(defun solve-nlopt-ik
  (msg)
  (let* ((str (send msg :data))
	 (kv (read-from-string (format nil "(~A)" str)))
	 ret)
    ;; flatten
    (while (and kv (not (keywordp (car kv))))
      (setq kv (car kv)))
    ;;
    (cond
     ((null kv)
      (nlopt-log "null args")
      (return-from solve-nlopt-ik nil)))
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
     ((or (not (subclassp (class (car kv)) coordinates))
	  (not (subclassp (class (cadr (member :move-target kv)))
			  cascaded-coords)))
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
	    (cadr (member :base-coords kv)))))
    ;;
    (setq ret
	  (apply #'fullbody-ik-test kv))
    ;;
    (ros::publish "/nlopt_ik_node/ik_response"
		  (instance std_msgs::string
			    :init
			    :data
			    (dump-object ret)))
    ))


(ros::subscribe "/nlopt_ik_node/ik_request"
		std_msgs::string
		#'solve-ik 1)
(ros::advertise "/nlopt_ik_node/ik_response"
                std_msgs::string)

