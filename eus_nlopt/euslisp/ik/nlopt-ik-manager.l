#!/usr/bin/env roseus

(defvar *nlopt_ik_node_name* "nlopt_ik_manager")
(defvar *algorithm* nil)
(require "nlopt-ik-node.l")

(if (not (and (boundp '*viewer*) *viewer*))
    (objects *robot*))

(defvar *ik-param*)
(defvar *best-cost* 1e+100)

(defun check-nlopt-ik-request
  (msg)
  (setq *best-cost* 1e+100)
  (setq *ik-param*
	(check-ik-param (send msg :data))))

(defun manage-nlopt-ik
  (msg)
  (let* ((str (send msg :data))
	 (kv (car (read-from-string (format nil "(~A)" str))))
	 ret cst
	 (mt (cadr (member :move-target *ik-param*)))
	 (tc (cadr (member :target-coords *ik-param*)))
	 (prev-av (copy-object (send *robot* :angle-vector)))
	 (prev-b (copy-object (send *robot* :worldcoords)))
	 (av (cdr (assoc :angle-vector kv)))
	 (wb (or (cdr (assoc :base-coords kv))
		 (cdr (assoc :worldcoords kv)))))
    (cond
     ((or (null mt)
	  (null tc)
	  (not (eq (length mt) (length tc))))
      (nlopt-log "invalid ik-param ~A" *ik-param*)
      (return-from manage-nlopt-ik nil))
     ((or (null av)
	  (null wb))
      (nlopt-log "invalid response ~A" str)
      (return-from manage-nlopt-ik nil)))
    (send *robot* :angle-vector av)
    (send *robot* :newcoords wb)
    (setq ret
	  (list
	   (cons :angle-vector av)
	   (cons :worldcoords wb)
	   (cons :dif
		 (mapcar
		  #'(lambda (mt tr)
		      (concatenate
		       float-vector
		       (scale 1e-3 (send mt :difference-position tr))
		       (send mt :difference-rotation tr)))
		  mt tc)))
	  )
    (setq cst
	  (norm
	   (apply
	    #'concatenate
	    (append
	     (list float-vector)
	     (cdr (assoc :dif ret))))))
    (cond
     ((> *best-cost* cst)
      (nlopt-log "publish ~A" str)
      (setq *best-cost* cst)
      (cond
       ((and (boundp '*viewer*) *viewer*)
	(send *irtviewer* :draw-objects)
	(x::window-main-one)))
      (ros::publish "/nlopt_ik_node/best_result"
		    (instance std_msgs::string
			      :init
			      :data
			      (dump-object ret))))
     (t
      (nlopt-log "trash ~A" str)
      (send *robot* :angle-vector prev-av)
      (send *robot* :newcoords prev-b))
     )))

(ros::subscribe "/nlopt_ik_node/ik_request"
		std_msgs::string
		#'check-nlopt-ik-request 1)
(ros::subscribe "/nlopt_ik_node/ik_response"
		std_msgs::string
		#'manage-nlopt-ik 1)
(ros::advertise "/nlopt_ik_node/best_result"
                std_msgs::string)

(ros::rate 10)
(do-until-key
 (cond
  ((not (ros::ok))
   (return-from nil nil)))
 (cond
  ((and (boundp '*viewer*) *viewer*)
   ;;(send *irtviewer* :draw-objects)
   (x::window-main-one)))
 (ros::spin-once)
 (ros::sleep))
