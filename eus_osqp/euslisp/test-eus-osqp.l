(load "package://eus_osqp/euslisp/eus-osqp.l")

;; QP example are obtained from http://www.me.titech.ac.jp/~mizu_lab/text/PDF-NLP/NLP1-QP-problem.pdf

;; min (x1-4)^2+x1x2+2(x2-3)^2
;;     =>x1^2-8x1+16+x1x2+2x2^2-12x2+18
;;       |x1 x2||a b;b c||x1 x2|'
;;       |ax1+bx2 bx1+cx2||x1 x2|'
;;       ax1^2+bx1x2+bx1x2+cx2^2
;;       a=1, c=2, b=0.5, q=|-8 -12|
;;     =>X=|x1 x2|, H=2*|1 1/2;1/2 2|
;; =>2.85714 2.28571
(defun demo-eus-osqp1
  (&key (solve-qp-mode :qp))
  "Demo for unbounded QP."
  (let* ((X0 (float-vector 0 0))
         (H (scale-matrix
             2
             (make-matrix
              2 2
              (list (list 1 0.5) (list 0.5 2)))))
         (q (float-vector -8 -12))
         (status #F(0))
         (ret (solve-osqp-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :solve-qp-mode solve-qp-mode
               :status status
               )))
    (if (= (round (elt status 0)) 1)
        (warn ";; solution ~A => ~A~%"
              ret
              (setq ret
                    (eps-v= (float-vector (/ 20.0 7.0) (/ 16.0 7.0)) ret)))
        (warn ";; qp error ~A => ~A~%"
              (round (elt status 0))
              (setq ret nil)))
    ;; correct solution
    ret
    ))

;;
;; min X'|2 1|X + |-8 -12|X
;;       |1 4|
;;     X=|x1 x2|'
;; s.t. |2 1|X = 2
;; =>-0.142857 2.28571
(defun demo-eus-osqp2
  (&key (solve-qp-mode :qp))
  "Demo for QP with equality constraint."
  (let* ((X0 (float-vector 10 10))
         (H (scale-matrix
             2
             (make-matrix
              2 2
              (list (list 1 0.5) (list 0.5 2)))))
         (q (float-vector -8 -12))
         (A (make-matrix
             1 2
             (list (list 2 1))))
         (B (float-vector 2))
         (status #F(0))
         (ret (solve-osqp-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :solve-qp-mode solve-qp-mode
               :status status
               )))
    (if (= (round (elt status 0)) 1)
        (warn ";; solution ~A => ~A~%"
              ret
              (setq ret
                    (eps-v= (float-vector (/ -1.0 7.0) (/ 16.0 7.0)) ret)))
        (warn ";; qp error ~A => ~A~%"
              (round (elt status 0))
              (setq ret nil)))
    ;; correct solution
    ret
    ))

;;
;; min X'|4 1|X + |-3 -4|X
;;       |1 2|
;;     X=|x1 x2|'
;; s.t. |1 2|X = 1
;;      X>=0
;; =>0.285714 0.357143
(defun demo-eus-osqp3
  (&key (solve-qp-mode :qp))
  "Demo for simply-bounded QP with equality constraint."
  (let* ((X0 (float-vector 10 10))
         (H (scale-matrix
             2
             (make-matrix
              2 2
              (list (list 4 1) (list 1 2)))))
         (q (float-vector -3 -4))
         (A (make-matrix
             1 2
             (list (list 1 2))))
         (B (float-vector 1))
         (LB (float-vector 0 0))
         (status #F(0))
         (ret (solve-osqp-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :state-min-vector LB
               :solve-qp-mode solve-qp-mode
               :status status
               )))
    (if (= (round (elt status 0)) 1)
        (warn ";; solution ~A => ~A~%"
              ret
              (setq ret
                    (eps-v= (float-vector (/ 1.0 7.0) (/ 3.0 7.0)) ret)))
        (warn ";; qp error ~A => ~A~%"
              (round (elt status 0))
              (setq ret nil)))
    ;; correct solution
    ret
    ))

;; In feasible
(defun demo-eus-osqp4
  (&key (solve-qp-mode :qp))
  "Demo for infeasible QP."
  (let* ((X0 (float-vector 0 0))
         (H (scale-matrix
             2
             (make-matrix
              2 2
              (list (list 1 0.5) (list 0.5 2)))))
         (q (float-vector -8 -12))
         ;; Infeasible lower bound and upper bound
         (LB (float-vector 10 10))
         (UB (float-vector -10 -10))
         ;; check-status = t => return value should be nil.
         (ret0 (solve-osqp-qp
                :initial-state x0
                :eval-weight-matrix H
                :eval-coeff-vector q
                :state-min-vector LB
                :state-max-vector UB
                :solve-qp-mode solve-qp-mode
                ))
         ;; check-status = nil => something is returned.
         (ret1 (solve-osqp-qp
                :initial-state x0
                :eval-weight-matrix H
                :eval-coeff-vector q
                :state-min-vector LB
                :state-max-vector UB
                :check-status nil
                :solve-qp-mode solve-qp-mode
                )))
    (warn ";; solution => ~A, ~A~%" ret0 ret1)
    ;; correct solution
    (and (not ret0) ret1)
    ))

;;
;; min X'|4 1 0 0 0 0 ...|X + |-3 -4 -3 -4 -3 -4 ...|X
;;       |1 2 0 0 0 0 ...|
;;       |0 0 4 1 0 0 ...|
;;       |0 0 1 2 0 0 ...|
;;       |0 0 0 0 4 1 ...|
;;       |0 0 0 0 1 2 ...|
;;       |: : : : : :    |
;;     X=|x1 x2 ... x300|'
;; s.t. |1 2 0 0 0 0 ...|X = |1|
;;      |0 0 1 2 0 0 ...|    |1|
;;      |0 0 0 0 1 2 ...|    |1|
;;      |: : : : : :    |    |:|
;;      X>=0
;; =>0.285714 0.357143 0.285714 0.357143 ...
(defun demo-eus-osqp5
  (&key (solve-qp-mode :qp))
  "Demo for QP with large matrix."
  (let* ((X0 (make-array 100 :element-type :float :initial-element 0.0))
         (H (scale-matrix
             2
             (let ((_H (make-array
                        (list 300 300)
                        :element-type float-vector
                        :initial-element 0)))
               (dotimes (i 150)
                 (setf (aref _H (* i 2) (* i 2)) 4)
                 (setf (aref _H (+ (* i 2) 1) (+ (* i 2) 1)) 2)
                 (setf (aref _H (+ (* i 2) 0) (+ (* i 2) 1)) 1)
                 (setf (aref _H (+ (* i 2) 1) (+ (* i 2) 0)) 1)
                 )
               _H)))
         (q (let ((_q (make-array 300 :element-type :float :initial-element 0.0)))
              (dotimes (i 150)
                (setelt _q (* i 2) -3)
                (setelt _q (+ (* i 2) 1) -4)
                )
              _q))
         (A (let ((_A (make-array
                       (list 150 300)
                       :element-type float-vector
                       :initial-element 0)))
              (dotimes (i 150)
                (setf (aref _A i (* i 2)) 1)
                (setf (aref _A i (+ (* i 2) 1)) 2)
                )
              _A))
         (B (make-array 150 :element-type :float :initial-element 1.0))
         (LB (make-array 300 :element-type :float :initial-element 0.0))
         (status #F(0))
         (ret (bench (solve-osqp-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :state-min-vector LB
               :solve-qp-mode solve-qp-mode
               :status status
               :eval-weight-matrix-sparce nil
               :equality-matrix-sparce nil
               :inequality-matrix-sparce nil
               ))))
    (if (= (round (elt status 0)) 1)
        (warn ";; solution ~A => ~A~%"
              ret
              (setq ret
                    (eps-v= (let ((_q (make-array 300 :element-type :float :initial-element 0.0)))
                              (dotimes (i 150)
                                (setelt _q (* i 2) (/ 1.0 7.0))
                                (setelt _q (+ (* i 2) 1) (/ 3.0 7.0))
                                )
                              _q) ret)))
        (warn ";; qp error ~A => ~A~%"
              (round (elt status 0))
              (setq ret nil)))
    ;; correct solution
    ret
    ))

;;
;; min X'|4 1         ...|X + |-3 -4 -3 -4 -3 -4 ...|X
;;       |1 2         ...|
;;       |    4 1     ...|
;;       |    1 2     ...|
;;       |        4 1 ...|
;;       |        1 2 ...|
;;       |: : : : : :    |
;;     X=|x1 x2 ... x300|'
;; s.t. |1 2         ...|X = |1|
;;      |    1 2     ...|    |1|
;;      |        1 2 ...|    |1|
;;      |: : : : : :    |    |:|
;;      X>=0
;; =>0.285714 0.357143 0.285714 0.357143 ...
(defun demo-eus-osqp6
  (&key (solve-qp-mode :qp))
  "Demo for QP with large sparce matrix."
  (let* ((X0 (make-array 300 :element-type :float :initial-element 0.0))
         (H (scale-matrix
             2
             (let ((_H (make-array
                        (list 300 300)
                        :element-type float-vector
                        :initial-element 0)))
               (dotimes (i 150)
                 (setf (aref _H (* i 2) (* i 2)) 4)
                 (setf (aref _H (+ (* i 2) 1) (+ (* i 2) 1)) 2)
                 (setf (aref _H (+ (* i 2) 0) (+ (* i 2) 1)) 1)
                 (setf (aref _H (+ (* i 2) 1) (+ (* i 2) 0)) 1)
                 )
               _H)))
         (Hsparce (let ((_Hsparce (make-array
                             (list 300 300)
                             :element-type float-vector
                             :initial-element 0)))
                    (dotimes (i 150)
                      (setf (aref _Hsparce (* i 2) (* i 2)) 1)
                      (setf (aref _Hsparce (+ (* i 2) 1) (+ (* i 2) 1)) 1)
                      (setf (aref _Hsparce (+ (* i 2) 0) (+ (* i 2) 1)) 1)
                      (setf (aref _Hsparce (+ (* i 2) 1) (+ (* i 2) 0)) 1)
                      )
                    _Hsparce))
         (q (let ((_q (make-array 300 :element-type :float :initial-element 0.0)))
              (dotimes (i 150)
                (setelt _q (* i 2) -3)
                (setelt _q (+ (* i 2) 1) -4)
                )
              _q))
         (A (let ((_A (make-array
                       (list 150 300)
                       :element-type float-vector
                       :initial-element 0)))
              (dotimes (i 150)
                (setf (aref _A i (* i 2)) 1)
                (setf (aref _A i (+ (* i 2) 1)) 2)
                )
              _A))
         (Asparce (let ((_Asparce (make-array
                       (list 150 300)
                       :element-type float-vector
                       :initial-element 0)))
              (dotimes (i 150)
                (setf (aref _Asparce i (* i 2)) 1)
                (setf (aref _Asparce i (+ (* i 2) 1)) 1)
                )
              _Asparce))
         (B (make-array 150 :element-type :float :initial-element 1.0))
         (LB (make-array 300 :element-type :float :initial-element 0.0))
         (status #F(0))
         (ret (bench (solve-osqp-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :state-min-vector LB
               :solve-qp-mode solve-qp-mode
               :status status
               :eval-weight-matrix-sparce Hsparce
               :equality-matrix-sparce Asparce
               ))))
    (if (= (round (elt status 0)) 1)
        (warn ";; solution ~A => ~A~%"
              ret
              (setq ret
                    (eps-v= (let ((_q (make-array 300 :element-type :float :initial-element 0.0)))
                              (dotimes (i 150)
                                (setelt _q (* i 2) (/ 1.0 7.0))
                                (setelt _q (+ (* i 2) 1) (/ 3.0 7.0))
                                )
                              _q) ret)))
        (warn ";; qp error ~A => ~A~%"
              (round (elt status 0))
              (setq ret nil)))
    ;; correct solution
    ret
    ))

;;
(defun demo-eus-osqp-all
    (&key (qp-mode :qp))
  "Demo for all QP and LP."
  (and (demo-eus-osqp1 :solve-qp-mode qp-mode)
       (demo-eus-osqp2 :solve-qp-mode qp-mode)
       (demo-eus-osqp3 :solve-qp-mode qp-mode)
       (demo-eus-osqp4 :solve-qp-mode qp-mode)
       (demo-eus-osqp5 :solve-qp-mode qp-mode)
       (demo-eus-osqp6 :solve-qp-mode qp-mode)
       ))

(warn ";; (demo-eus-osqp1) ;; ~A~%" (documentation 'demo-eus-osqp1))
(warn ";; (demo-eus-osqp2) ;; ~A~%" (documentation 'demo-eus-osqp2))
(warn ";; (demo-eus-osqp3) ;; ~A~%" (documentation 'demo-eus-osqp3))
(warn ";; (demo-eus-osqp4) ;; ~A~%" (documentation 'demo-eus-osqp4))
(warn ";; (demo-eus-osqp5) ;; ~A~%" (documentation 'demo-eus-osqp5))
(warn ";; (demo-eus-osqp6) ;; ~A~%" (documentation 'demo-eus-osqp6))
(warn ";; (demo-eus-osqp-all) ;; ~A~%" (documentation 'demo-eus-osqp-all))
