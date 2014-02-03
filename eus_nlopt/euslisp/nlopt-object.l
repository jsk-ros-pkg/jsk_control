
(require "nlopt.l")

(defvar *nlopt-object*)

(defun-c-callable nlopt-f ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp
	   (list (send *nlopt-object*
		       :evaluation-function
		       v1 v2)
		 0)))

(defun-c-callable nlopt-df ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp
	   (list (send *nlopt-object*
		       :evaluation-function-gradient
		       v1 v2)
		 0)))

(defun-c-callable nlopt-g ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp
	   (list (send *nlopt-object*
		       :equality-function
		       v1 v2)
		 0)))

(defun-c-callable nlopt-dg ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp
	   (list (send *nlopt-object*
		       :equality-function-gradient
		       v1 v2)
		 0)))

(defun-c-callable nlopt-h ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp
	   (list (send *nlopt-object*
		       :inequality-function
		       v1 v2)
		 0)))

(defun-c-callable nlopt-dh ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp
	   (list (send *nlopt-object*
		       :inequality-function-gradient
		       v1 v2)
		 0)))

(defclass nlopt-object
  :super object
  :slots (algorithm
	  initial-state
	  state-min-vector
	  state-max-vector
	  ftol
	  xtol
	  eqthre
	  max-eval
	  max-time
	  state-dimension
	  equality-dimension
	  inequality-dimension
	  log?
	  ))

(defmethod nlopt-object
  (:init
   (&rest
    args
    &key
    (algorithm SLSQP)
    ((:initial-state initial-state) #F())
    ((:state-min-vector state-min-vector)
     (map float-vector #'(lambda (val) -1e+5) initial-state))
    ((:state-max-vector state-max-vector)
     (map float-vector #'(lambda (val) 1e+5) initial-state))
    (ftol 1e-8)
    (xtol 1e-8)
    (eqthre 1e-3)
    (max-eval -1)
    (max-time -1)
    (state-dimension (length initial-state))
    (equality-dimension 0)
    (inequality-dimension 0)
    (log? t)
    &allow-other-keys
    )
   (send self :algorithm algorithm)
   (send self :initial-state initial-state)
   (send self :state-min-vector state-min-vector)
   (send self :state-max-vector state-max-vector)
   (send self :ftol ftol)
   (send self :xtol xtol)
   (send self :eqthre eqthre)
   (send self :max-time max-time)
   (send self :max-eval max-eval)
   (send self :state-dimension state-dimension)
   (send self :equality-dimension equality-dimension)
   (send self :inequality-dimension inequality-dimension)
   (send self :log? log?)
   (setq *nlopt-object* self)
   )
  ;;
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  (:optimize
   (&rest args)
   (apply
    #'nlopt-optimize
    (append
     args
     (list
      :alg algorithm
      :initial-state initial-state
      :state-min-vector state-min-vector
      :state-max-vector state-max-vector
      :evaluation-function 'nlopt-f
      :evaluation-function-gradient 'nlopt-df
      :equality-function 'nlopt-g
      :equality-function-gradient 'nlopt-dg
      :inequality-function 'nlopt-h
      :inequality-function-gradient 'nlopt-dh
      :ftol 1e-8 :xtol 1e-8 :eqthre 1e-4
      :max-eval max-eval :max-time max-time
      :log? log?
      :state-dimension state-dimension
      :equality-dimension equality-dimension
      :inequality-dimension inequality-dimension))))
  (:evaluation-function (v1 v2) 0) ;; implement here
  (:equality-function (v1 v2) 0) ;; g = 0
  (:inequality-function (v1 v2) 0) ;; h <= 0
  (:evaluation-function-gradient (v1 v2) 0)
  (:equality-function-gradient (v1 v2) 0)
  (:inequality-function-gradient (v1 v2) 0)
  (:simple-jacobian
   (func v1 v2
	 &rest
	 args)
   (let* ((w (/ (length v2) (length v1)))
	  (h (length v1))
	  (buf (instantiate float-vector w))
	  tmp
	  (org (instantiate float-vector w))
	  (delta 1e-6))
     (apply #'send (append (list self func v1 org) args))
     (dotimes (i h)
       (setq tmp (aref v1 i))
       (setf (aref v1 i) (+ (aref v1 i) delta))
       ;;
       (apply #'send (append (list self func v1 buf) args))
       (dotimes (j w)
	 (setf (aref v2 (+ j (* i w)))
	       (/ (- (aref buf j) (aref org j)) delta)))
       ;;
       (setf (aref v1 i) tmp))))
  )
