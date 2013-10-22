
(require "nlopt.l")

(defvar *nlopt-object*)

(defun-c-callable nlopt-f ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp (list (send *nlopt-object* :f v1 v2) 0)))

(defun-c-callable nlopt-df ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp (list (send *nlopt-object* :df v1 v2) 0)))

(defun-c-callable nlopt-g ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp (list (send *nlopt-object* :g v1 v2) 0)))

(defun-c-callable nlopt-dg ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp (list (send *nlopt-object* :dg v1 v2) 0)))

(defun-c-callable nlopt-h ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp (list (send *nlopt-object* :h v1 v2) 0)))

(defun-c-callable nlopt-dh ((v1 :string) (v2 :string)) :integer
  (find-if #'numberp (list (send *nlopt-object* :dh v1 v2) 0)))

(defclass nlopt-object
  :super object
  :slots (algorithm
	  x0
	  x-min
	  x-max
	  ftol
	  xtol
	  eqthre
	  max-eval
	  max-time
	  m-x
	  m-g
	  m-h
	  log?
	  ))

(defmethod nlopt-object
  (:init
   (&rest
    args
    &key
    (algorithm SLSQP)
    (x0 #F())
    (x-min (map float-vector #'(lambda (val) -1e+5) x0))
    (x-max (map float-vector #'(lambda (val) 1e+5) x0))
    (ftol 1e-8)
    (xtol 1e-8)
    (eqthre 1e-3)
    (max-eval -1)
    (max-time -1)
    (m-x (length x0))
    (m-g 0)
    (m-h 0)
    (log? t)
    &allow-other-keys
    )
   (send self :algorithm algorithm)
   (send self :x0 x0)
   (send self :x-min x-min)
   (send self :x-max x-max)
   (send self :ftol ftol)
   (send self :xtol xtol)
   (send self :eqthre eqthre)
   (send self :max-time max-time)
   (send self :max-eval max-eval)
   (send self :m-x m-x)
   (send self :m-g m-g)
   (send self :m-h m-h)
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
      ((cadr args)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  (:optimize
   (&rest args)
   (apply
    #'nlopt-optmize
    (append
     args
     (list
      :alg algorithm
      :x0 x0
      :x-min x-min
      :x-max x-max
      :f 'nlopt-f :df 'nlopt-df
      :g 'nlopt-g :dg 'nlopt-dg
      :h 'nlopt-h :dh 'nlopt-dh
      :ftol 1e-8 :xtol 1e-8 :eqthre 1e-4
      :max-eval max-eval :max-time max-time
      :log? log?
      :m-x m-x
      :m-g m-g
      :m-h m-h))))
  (:f (v1 v2) 0)
  (:g (v1 v2) 0)
  (:h (v1 v2) 0)
  (:df (v1 v2) 0)
  (:dg (v1 v2) 0)
  (:dh (v1 v2) 0)
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
