
(defvar *eiquadprog-plugin*
  (labels
      ((library_search
	(str &key
	     (depth 0)
	     colon-pos lib-path)
	(format t "  [~A] target=" depth)
	(cond
	 ((eq (length str) 0)
	  (format t "~% qp_plugin_not_found in eiquadprog.l~%")
	  (exit -1))
	 ((and (setq colon-pos (or (position #\: str) (length str)))
	       (setq lib-path (subseq str 0 colon-pos))
	       (setq lib-path
		     (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
			 (subseq lib-path 0 (- (length lib-path) 1))
		       lib-path))
	       (probe-file (setq lib-path
				 (print (format nil "~A/libeus_qp.so" lib-path)))))
	  (load-foreign lib-path))
	 (t
	  (library_search (subseq str (min (length str) (+ colon-pos 1)))
			  :depth (+ depth 1))))))
    (library_search (format nil "~A:~A/lib"
			    (unix:getenv "LD_LIBRARY_PATH")
			    (read-line (piped-fork "rospack find eus_qp"))))))

(defforeign _solve-eiquadprog
  *eiquadprog-plugin*
  "solve_eiquadprog"
  (:string :string ;; min xTGx/2 + g0x
	   :string :string ;; CEx + ce = 0
	   :string :string ;; CIx + ci > 0
	   :string ;; ret
	   :integer ;; |x|
	   :integer ;; |ce|
	   :integer ;: |ci|
	   :double ;; eqthre	   :
	   :integer ;; debug?
	   :string :string :string ;; ret, ce, ci values
	   )
  :string)

(defforeign _get_constraints_check_flag
  *eiquadprog-plugin*
  "get_constraints_check_flag"
  ()
  :integer)

;; util
(defun my-copy-matrix (mat vec)
  (let ((buf (if (matrixp mat) (cdr (assoc 'entity (send mat :slots))) mat)))
    (dotimes (i (length vec))
      (setf (aref vec i) (aref buf i)))))

(defun solve-eiquadprog-raw
  (&key
   (x0 (float-vector 0 0))
   (ce0 (float-vector 0))
   (ci0 (float-vector 0))
   (x_len (length x0))
   (ce_len (length ce0))
   (ci_len (length ci0))
   ;;
   (G (unit-matrix x_len))
   (g0 (instantiate float-vector x_len))
   (CE (make-matrix ce_len x_len))
   (CI (make-matrix ci_len x_len))
   ;;
   (debug? nil)
   (eqthre 1e-1)
   (ret_buf #F(0))
   (ce_err (instantiate float-vector ce_len))
   (ci_err (instantiate float-vector ci_len))
   ret
   &allow-other-keys)
  (setq
   ret
   (_solve-eiquadprog
    (send G :get-val 'entity)
    g0
    (send CE :get-val 'entity)
    ce0
    (send CI :get-val 'entity)
    ci0
    x0 x_len ce_len ci_len
    eqthre
    (if debug? 1 0)
    ret_buf ce_err ci_err
    ))
  ret
  )

;; always there will be some answers
;;
;; min xTGx/2 + g0x + eeT Gee ee + eiT Gei ei
;;   CEx + ce + ee = 0
;;   CIx + ci + ei > 0
;;
(defun solve-eiquadprog-raw-with-error
  (&key
   (x0 (float-vector 0 0))
   (ce0 (float-vector 0))
   (ci0 (float-vector 0))
   (x_len (length x0))
   (ce_len (length ce0))
   (ci_len (length ci0))
   ;;
   (G (unit-matrix x_len))
   (g0 (instantiate float-vector x_len))
   (CE (make-matrix ce_len x_len))
   (CI (make-matrix ci_len x_len))
   ;;
   (ee (instantiate float-vector (length ce0)))
   (ei (instantiate float-vector (length ci0)))
   (x0+ee+ei (concatenate float-vector x0 ee ei))
   (error-gain 1e+6)
   (Gee (scale-matrix error-gain (unit-matrix (length ee))))
   (Gei (scale-matrix error-gain (unit-matrix (length ei))))
   (g0+0 (concatenate
	  float-vector
	  g0
	  (instantiate float-vector (+ (length ee) (length ei)))))
   (G+Gee+Gei (matrix-append (list G Gee Gei) '(1 1)))
   (CE+I+0 (matrix-append
	    (list CE
		  (unit-matrix (length ce0))
		  (make-matrix (length ce0) (length ei)))
	    '(0 1)))
   (CI+0+I (matrix-append
	    (list CI
		  (make-matrix (length ci0) (length ee))
		  (unit-matrix (length ci0)))
	    '(0 1)))
   ;;
   (debug? nil)
   (eqthre 1e-1)
   (ret_buf #F(0))
   (ce_err (instantiate float-vector ce_len))
   (ci_err (instantiate float-vector ci_len))
   ret
   &allow-other-keys)
  (setq
   ret
   (_solve-eiquadprog
    (send G+Gee+Gei :get-val 'entity)
    g0+0
    (send CE+I+0 :get-val 'entity)
    ce0
    (send CI+0+I :get-val 'entity)
    ci0
    x0+ee+ei
    (length x0+ee+ei) ce_len ci_len
    eqthre
    (if debug? 1 0)
    ret_buf ce_err ci_err
    ))
  (dotimes (i (length x0)) (setf (aref x0 i) (aref x0+ee+ei i)))
  x0
  )

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
  (if m-list
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
	ret)
    #2f()))

(defun solve-eiquadprog
  (&key
   (initial-state #F(0))
   (eval-weight-matrix (unit-matrix (length initial-state)))
   (eval-coeff-vector (instantiate float-vector (length initial-state)))
   (equality-matrix #2f())
   (equality-vector #f())
   (state-min-vector #F())
   (state-max-vector #F())
   (inequality-matrix #2f())
   (inequality-min-vector #F())
   (inequality-max-vector #F())
   (timer (instance mtimer :init))
   (timebuf #F(0))
   (debug? nil)
   ;;
   (eqthre 1e-1)
   (ret_buf #F(0))
   ;;(ce_err (instantiate float-vector (length ))
   ;;(ci_err (instantiate float-vector ci_len))
   (ok? #i(-1))
   (eiquadprog-function 'solve-eiquadprog-raw)
   )
  (let* ((x0 initial-state)
	 (G eval-weight-matrix)
	 (g0 eval-coeff-vector)
	 (CE equality-matrix)
	 (ce0 (scale -1 equality-vector))
	 (CI
	  (matrix-append
	   (flatten
	    (list
	     (if (plusp (length state-min-vector))
		 (unit-matrix (length state-min-vector)))
	     (if (plusp (length state-max-vector))
		 (scale-matrix -1 (unit-matrix (length state-max-vector))))
	     (if (plusp (length inequality-min-vector))
		 inequality-matrix)
	     (if (plusp (length inequality-max-vector))
		 (scale-matrix -1 inequality-matrix))))
	   '(1 0)))
	 (ci0
	  (concatenate
	   float-vector
	   (scale -1 state-min-vector)
           state-max-vector
	   (scale -1 inequality-min-vector)
	   inequality-max-vector))
	 ret)
    (setq
     ret
     (funcall
      eiquadprog-function
      :x0 x0 :G G :g0 g0 :CE CE :ce0 ce0 :CI CI :ci0 ci0
      :eqthre eqthre
      :ret_buf ret_buf ;;:ce_err ce_err :ci_err ci_err
      :debug? debug?))
    (setf (aref timebuf 0) (send timer :stop))
    (if debug? (format t "TIME: ~A sec~%" (aref timebuf 0)))
    (setf (aref ok? 0) (_get_constraints_check_flag))
    (if (= (aref ok? 0) 0) ret)
    ))

;;;;
;;  f >= 6.4
;;;;
(defun test-eiquadprog
  (&rest args)
  ;; (apply
  ;;  #'solve-eiquadprog-raw
  ;;  (append
  ;;   (list :x0 (instantiate float-vector 3)
  ;; 	  :G (make-matrix 3 3 '((2.1 0.0 1.0) (1.5 2.2 0.0) (1.2 1.3 3.1)))
  ;; 	  :g0 (float-vector 6.0 1.0 1.0)
  ;; 	  :CE 	(make-matrix 1 3 '((1.0 2.0 -1.0)))
  ;; 	  :ce0 (float-vector -4)
  ;; 	  :CI (make-matrix
  ;; 	       4 3
  ;; 	       '((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0) (-1.0 -1.0 0.0)))
  ;; 	  :ci0 (float-vector 0 0 0 10))
  ;;   args))
  (apply
   #'solve-eiquadprog
   (append
    (list :initial-state (instantiate float-vector 3)
	  :eval-weight-matrix
	  (make-matrix 3 3 '((2.1 0.0 1.0) (1.5 2.2 0.0) (1.2 1.3 3.1)))
	  :eval-coeff-vector (float-vector 6.0 1.0 1.0)
	  :equality-matrix (make-matrix 1 3 '((1.0 2.0 -1.0)))
	  :equality-vector (float-vector 4)
	  :inequality-matrix
	  (make-matrix
	   4 3
	   '((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0) (-1.0 -1.0 0.0)))
	  :inequality-min-vector (float-vector 0 0 0 -10)
	  :debug? t)
    args))
  )
