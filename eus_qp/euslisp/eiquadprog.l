(defvar m (load-foreign "../lib/libeus_qp.so"))

(defforeign _solve-eiquadprog m "solve_eiquadprog"
  (:string :string ;; min xTGx/2 + g0x
	   :string :string ;; CEx + ce = 0
	   :string :string ;; CIx + ci > 0
	   :string ;; ret
	   :integer ;; |x|
	   :integer ;; |ce|
	   :integer ;: |ci|
	   :integer ;; debug?
	   )
  :string)

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
    (if debug? 1 0)
    ))
  ret
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
   (f0 #F(0))
   (eval-weight-matrix (unit-matrix (length f0)))
   (eval-coeff-vector (scale 0 f0))
   (eq-matrix #2f())
   (eq-vector #f())
   (f0-min #F())
   (f0-max #F())
   (non-eq-matrix #2f())
   (non-eq-min-vector #F())
   (non-eq-max-vector #F())
   (timer (instance mtimer :init))
   (timebuf #F(0))
   (debug? nil))
  (let* ((x0 f0)
	 (G eval-weight-matrix)
	 (g0 eval-coeff-vector)
	 (CE eq-matrix)
	 (ce0 (scale -1 eq-vector))
	 (CI
	  (matrix-append
	   (flatten
	    (list
	     (if (plusp (length f0-min))
		 (unit-matrix (length f0-min)))
	     (if (plusp (length f0-max))
		 (scale-matrix -1 (unit-matrix (length f0-max))))
	     (if (plusp (length non-eq-min-vector))
		 non-eq-matrix)
	     (if (plusp (length non-eq-max-vector))
		 (scale-matrix -1 non-eq-matrix))))
	   '(1 0)))
	 (ci0
	  (concatenate
	   float-vector
	   (scale -1 f0-min)
	   f0-max
	   (scale -1 non-eq-min-vector)
	   non-eq-max-vector))
	 ret)
    (setq
     ret
     (solve-eiquadprog-raw
      :x0 x0 :G G :g0 g0 :CE CE :ce0 ce0 :CI CI :ci0 ci0 :debug? debug?))
    (setf (aref timebuf 0) (send timer :stop))
    (if debug? (format t "TIME: ~A sec~%" (aref timebuf 0)))
    ret
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
    (list :f0 (instantiate float-vector 3)
	  :eval-weight-matrix
	  (make-matrix 3 3 '((2.1 0.0 1.0) (1.5 2.2 0.0) (1.2 1.3 3.1)))
	  :eval-coeff-vector (float-vector 6.0 1.0 1.0)
	  :eq-matrix (make-matrix 1 3 '((1.0 2.0 -1.0)))
	  :eq-vector (float-vector 4)
	  :non-eq-matrix
	  (make-matrix
	   4 3
	   '((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0) (-1.0 -1.0 0.0)))
	  :non-eq-min-vector (float-vector 0 0 0 -10)
	  :debug? t)
    args))
  )
