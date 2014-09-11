(load "package://eus_qpoases/euslisp/eus-qpoases.l")

;; QP example are obtained from http://www.me.titech.ac.jp/~mizu_lab/text/PDF-NLP/NLP1-QP-problem.pdf

;; min (x1-4)^2+x1x2+2(x2-3)^2
;;     =>x1^2-8x1+16+x1x2+2x2^2-12x2+18
;;       |x1 x2||a b;b c||x1 x2|'
;;       |ax1+bx2 bx1+cx2||x1 x2|'
;;       ax1^2+bx1x2+bx1x2+cx2^2
;;       a=1, c=2, b=0.5, q=|-8 -12|
;;     =>X=|x1 x2|, H=2*|1 1/2;1/2 2|
;; =>2.85714 2.28571
(defun demo-eus-qpOASES1
  ()
  (let* ((X0 (float-vector 0 0))
         (H (scale-matrix
             2
             (make-matrix
              2 2
              (list (list 1 0.5) (list 0.5 2)))))
         (q (float-vector -8 -12))
         (ret (solve-qpoases
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q)))
    (warn ";; solution ~A => ~A~%"
          ret
          (eps-v= (float-vector (/ 20.0 7.0) (/ 16.0 7.0)) ret)) ;; correct solution
    ))

;;
;; min X'|2 1|X + |-8 -12|X
;;       |1 4|
;;     X=|x1 x2|'
;; s.t. |2 1|X = 2
;; =>-0.142857 2.28571
(defun demo-eus-qpOASES2
  ()
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
         (ret (solve-qpoases
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B)))
    (warn ";; solution ~A => ~A~%"
          ret
          (eps-v= (float-vector (/ -1.0 7.0) (/ 16.0 7.0)) ret)) ;; correct solution
    ))

;;
;; min X'|4 1|X + |-3 -4|X
;;       |1 2|
;;     X=|x1 x2|'
;; s.t. |1 2|X = 1
;;      X>=0
;; =>0.285714 0.357143
(defun demo-eus-qpOASES3
  ()
  (let* ((X0 (float-vector 10 10))
         (H (make-matrix
             2 2
             (list (list 4 1) (list 1 2))))
         (q (float-vector -3 -4))
         (A (make-matrix
             1 2
             (list (list 1 2))))
         (B (float-vector 1))
         (LB (float-vector 0 0))
         (ret (solve-qpoases
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :state-min-vector LB)))
    (warn ";; solution ~A => ~A~%"
          ret
          (eps-v= (float-vector (/ 2.0 7.0) (/ 5.0 14.0)) ret)) ;; correct solution
    ))

(warn ";; (demo-eus-qpOASES1)~%")
(warn ";; (demo-eus-qpOASES2)~%")
(warn ";; (demo-eus-qpOASES3)~%")
