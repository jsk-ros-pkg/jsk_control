#!/usr/bin/env roseus

(defun space-str (id)
  (coerce (make-list id :initial-element #\ ) string))

;; gen-config-string '(layer ((name "input") (type "memorydata") (top "input") (memory_data_param ((batch 19)))))
(defun gen-config-string
  (dl &optional
      (depth 0)
      (spc (space-str depth)))
  (cond
   ((null dl) nil)
   ((stringp dl) (list (format nil "~A\"~A\"" spc dl)))
   ((atom dl) (list (format nil "~A~A" spc dl)))
   ((listp (car dl))
    (apply 'append
	   (mapcar #'(lambda (_dl) (gen-config-string _dl depth))
		   dl)))
   ((and (symbolp (car dl))
	 (numberp (cadr dl)))
    (list (format nil "~A~A: ~A" spc (car dl) (cadr dl))))
   ((and (symbolp (car dl))
	 (stringp (cadr dl)))
    (list (format nil "~A~A: \"~A\"" spc (car dl) (cadr dl))))
   ((and (symbolp (car dl))
	 (listp (cadr dl)))
    (append
     (list (format nil "~A~A {" spc (car dl)))
     (apply 'append
	    (mapcar
	     #'(lambda (_dl) (gen-config-string _dl (+ depth 1)))
	     (cdr dl)))
     (list (format nil "~A}" spc))
     ))
   (t
    (warning-message 1 "undefined config ~A~%" dl))
   ))

(defun gen-simple-layer-list
  (name type bottoms tops)
  (append
   (list 'layer)
   (list (list 'name name))
   (list (list 'type type))
   (mapcar '(lambda (bottom) (list 'bottom bottom)) (flatten (list bottoms)))
   (mapcar '(lambda (top) (list 'top top)) (flatten (list tops)))
   ))

(defun gen-memory-layer-string
  (name top1 top2 batch_size channels)
  (gen-config-string
   (append
    (gen-simple-layer-list
     name "MemoryData" nil (list top1 top2))
    (list (list 'memory_data_param
		(list 'batch_size batch_size)
		(list 'channels channels)
		(list 'height 1)
		(list 'width 1))))))

(defun gen-ip-layer-string
  (name bottom top num)
  (gen-config-string
   (append
    (gen-simple-layer-list
     name "InnerProduct" bottom top)
    (list
     (list 'param
	   (list 'lr_mult 1)
	   (list 'decay_mult 1))
     (list 'inner_product_param
	   (list 'num_output num)
	   (list 'weight_filler
		 (list 'type "gaussian")
		 (list 'std 1)
		 (list 'sparse 1))
	   (list 'bias_filler
		 (list 'type "constant")
		 (list 'value 0)))))))

(defun gen-sigmoid-layer-string
  (name bottom top)
  (gen-config-string
   (gen-simple-layer-list name "Sigmoid" bottom top)))

(defun gen-reshape-layer-string
  (name bottom top)
  (gen-config-string
   (append
    (gen-simple-layer-list name "Reshape" bottom top)
    (list
     (list 'reshape_param
	   (list 'shape
		 (list 'dim 0)
		 (list 'dim -1))))
    )))

(defun gen-loss-layer-string
  (name bottom1 bottom2 top)
  (gen-config-string
   (gen-simple-layer-list name "EuclideanLoss" (list bottom1 bottom2) top)))

(defun gen-concate-layer-string
  (name bottom1 bottom2 top)
  (gen-config-string
   (gen-simple-layer-list name "Concat" (list bottom1 bottom2) top)))

(defun gen-redundancy-ik-net
  (s m1 d1 i k d2 m2)
  (if (numberp m1) (setq m1 (make-list d1 :initial-element m1)))
  (if (numberp m2) (setq m2 (make-list d2 :initial-element m2)))
  (let* ((id 0) tmp)
    (flatten
     (list
      (format nil
	      "name: \"redundancy_ik_net_~As_~Am1_~Ad1_~Ai_~Ak_~Ad2_~Am2\""
	      s (car m1) d1 i k d2 (car m2))
      (gen-memory-layer-string
       "input" "input" "__dummy_label1" 32 s)
      ;;
      (mapcar
       #'(lambda (_m1)
	   (setq
	    tmp
	    (list
	     (gen-ip-layer-string
	      (format nil "ip~A" id)
	      (if (eq id 0) "input" (format nil "sig~A" id))
	      (format nil "ip~A" id)
	      _m1)
	     (gen-sigmoid-layer-string
	      (format nil "sig~A" id)
	      (format nil "ip~A" id)
	      (format nil "sig~A" (+ 1 id)))))
	   (incf id)
	   tmp)
       m1)
      ;;
      (gen-ip-layer-string
       "redundancy" (format nil "sig~A" id) "redundancy" i)
      (gen-sigmoid-layer-string
       "normalized_redundancy" "redundancy" "normalized_redundancy")
      (gen-sigmoid-layer-string
       "_debug_redundancy" "redundancy" "_debug_redundancy")
      ;;
      (gen-memory-layer-string
       "target" "_target" "__dummy_label2" 32 k)
      (gen-reshape-layer-string "target_reshape" "_target" "target")
      ;;
      (gen-concate-layer-string "concate"
				"target" "normalized_redundancy"
				(format nil "sig~A" (incf id)))
      ;;
      (mapcar
       #'(lambda (_m2)
	   (setq
	    tmp
	    (list
	     (gen-ip-layer-string
	      (format nil "ip~A" id)
	      (format nil "sig~A" id)
	      (format nil "ip~A" id)
	      _m2)
	     (gen-sigmoid-layer-string
	      (format nil "sig~A" id)
	      (format nil "ip~A" id)
	      (format nil "sig~A" (+ 1 id)))))
	   (incf id)
	   tmp)
       m2)
      ;;
      (gen-ip-layer-string "output"
			   (format nil "sig~A" id)
			   "output" s)
      (gen-loss-layer-string
       "loss" "output" "input" "loss")
      ))))

(defun gen-predict-ik-net
  (s m1 d1 i k d2 m2)
  (if (numberp m1) (setq m1 (make-list d1 :initial-element m1)))
  (if (numberp m2) (setq m2 (make-list d2 :initial-element m2)))
  (let* ((id 0) tmp)
    (setq id (+ 1 d1))
    (flatten
     (list
      (format nil
	      "name: \"redundancy_ik_net_~As_~Am1_~Ad1_~Ai_~Ak_~Ad2_~Am2\""
	      s (car m1) d1 i k d2 (car m2))
      (format nil "input: \"input\"")
      (format nil "input_dim: 1 ## num")
      (format nil "input_dim: ~A ## channel" (+ i k))
      (format nil "input_dim: 1 ## w")
      (format nil "input_dim: 1 ## h")
      ;;
      (mapcar
       #'(lambda (_m2)
	   (setq
	    tmp
	    (list
	     (gen-ip-layer-string
	      (format nil "ip~A" id)
	      (if (eq id (+ 1 d1)) "input" (format nil "sig~A" id))
	      (format nil "ip~A" id)
	      _m2)
	     (gen-sigmoid-layer-string
	      (format nil "sig~A" id)
	      (format nil "ip~A" id)
	      (format nil "sig~A" (+ 1 id)))))
	   (incf id)
	   tmp)
       m2)
      ;;
      (gen-ip-layer-string "output"
			   (format nil "sig~A" id)
			   "output" s)
      ))))


(defun fileoutput-layer-string
  (path lstr)
  (let* ((p (open path :direction :output)))
    (dolist (str lstr)
      (format p "~A~%" str))
    (close p)))
;; fileoutput-layer-string "test.prototxt" (gen-redundancy-ik-net 9 100 3 3 6 3 100)