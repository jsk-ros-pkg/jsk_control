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
  (&key (solve-qp-mode :qp))
  "Demo for unbounded QP."
  (let* ((X0 (float-vector 0 0))
         (H (scale-matrix
             2
             (make-matrix
              2 2
              (list (list 1 0.5) (list 0.5 2)))))
         (q (float-vector -8 -12))
         (ret (solve-qpoases-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :solve-qp-mode solve-qp-mode)))
    (warn ";; solution ~A => ~A~%"
          ret
          (setq ret
		(eps-v= (float-vector (/ 20.0 7.0) (/ 16.0 7.0)) ret)))
    ;; correct solution
    ret
    ))

;;
;; min X'|2 1|X + |-8 -12|X
;;       |1 4|
;;     X=|x1 x2|'
;; s.t. |2 1|X = 2
;; =>-0.142857 2.28571
(defun demo-eus-qpOASES2
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
         (ret (solve-qpoases-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :solve-qp-mode solve-qp-mode)))
    (warn ";; solution ~A => ~A~%"
          ret
          (setq ret (eps-v= (float-vector (/ -1.0 7.0) (/ 16.0 7.0)) ret)))
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
(defun demo-eus-qpOASES3
  (&key (solve-qp-mode :qp))
  "Demo for simply-bounded QP with equality constraint."
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
         (ret (solve-qpoases-qp
               :initial-state x0
               :eval-weight-matrix H
               :eval-coeff-vector q
               :equality-matrix A
               :equality-vector B
               :state-min-vector LB
               :solve-qp-mode solve-qp-mode)))
    (warn ";; solution ~A => ~A~%"
          ret
          (setq ret (eps-v= (float-vector (/ 2.0 7.0) (/ 5.0 14.0)) ret)))
    ;; correct solution
    ret
    ))

;; In feasible
(defun demo-eus-qpOASES4
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
         (ret0 (solve-qpoases-qp
                :initial-state x0
                :eval-weight-matrix H
                :eval-coeff-vector q
                :state-min-vector LB
                :state-max-vector UB
                :solve-qp-mode solve-qp-mode
                ))
         ;; check-status = nil => something is returned.
         (ret1 (solve-qpoases-qp
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

;; sample for lp
;;max 5x1 + 4x2
;;    => [5 4] [x1 x2]'
;;s.t x1 + 2x2 ≤ 80
;;    4x1 + 4x2 ≤ 180
;;    3x1 + x2 ≤ 90
;;    x1 ≥ 0, x2 ≥ 0
;;    =>
;;    |1 2||x1 x2|'<=80
;;    |4 4||x1 x2|'<=180
;;    |3 1||x1 x2|'<=90
;;    |x1 x2|>=0
;;=>[22.5 22.5]
(defun demo-eus-qpOASES5
  (&key (solve-lp-mode :lp))
  "Demo for LP with inequality."
  (let* ((X0 (float-vector 0 0 0 0))
         (q (scale -1 (float-vector 5 4)))
         (A (make-matrix
             3 2
             (list (list 1 2)
                   (list 4 4)
                   (list 3 1))))
         (ubA (float-vector 80 180 90))
         (ret (solve-qpoases-lp
               :initial-state x0
               :eval-coeff-vector q
               :state-min-vector (float-vector 0 0)
               :inequality-matrix A
               :inequality-max-vector ubA
               :solve-lp-mode solve-lp-mode
               )))
    (warn ";; solution ~A => ~A~%"
          ret
          (setq ret
                (eps-v= (float-vector 22.5 22.5) ret)))
    ;; correct solution
    ret
    ))

;; min −x1 x−2
;;     =>|-1 -1 0 0||x1 x2 x3 x4|'
;; s.t. 2x1 + 3x2 + x3 = 12
;;      −x1 + x2 − x4 = −2
;;      x1 ≥ 0, x2 ≥ 0, x3 ≥ 0, x4 ≥ 0
;;      =>
;;      |2  3 1  0|X = 12
;;      |-1 1 0 -1|X =-2
;;      X>=0
;; =>[18/5, 8/5, 0, 0]
(defun demo-eus-qpOASES6
  (&key (solve-lp-mode :lp))
  "Demo for LP with equality."
  (let* ((X0 (float-vector 0 0 0 0))
         (q (float-vector -1 -1 0 0))
         (A (make-matrix
             2 4
             (list (list 2 3 1 0)
                   (list -1 1 0 -1))))
         (ubA (float-vector 12 -2))
         (ret (solve-qpoases-lp
               :initial-state x0
               :eval-coeff-vector q
               :state-min-vector (float-vector 0 0 0 0)
               :equality-matrix A
               :equality-vector ubA
               :solve-lp-mode solve-lp-mode
               )))
    (warn ";; solution ~A => ~A~%"
          ret
          (setq ret
                (eps-v= (float-vector (/ 18.0 5) (/ 8.0 5) 0 0) ret)))
    ;; correct solution
    ret
    ))

;; min  x1 + x2
;;      => |1 1 0||x1 x2 x3|'
;; s.t. 2x1 − x2 − x3 = 4
;;      x2 ≥ 0, x3 ≥ 0
;;      =>|2 -1 -1|X=4
;;        |-inf 0 0|<=X<=|inf inf inf|
;; =>[2.0 0.0 0.0]
(defun demo-eus-qpOASES7
  (&key (solve-lp-mode :lp))
  "Demo for LP with equality + unbound."
  (let* ((X0 (float-vector 0 0 0 0))
         (q (float-vector 1 1 0))
         (A (make-matrix
             1 3
             (list (list 2 -1 -1))))
         (ubA (float-vector 4))
         (ret (solve-qpoases-lp
               :initial-state x0
               :eval-coeff-vector q
               :state-min-vector (float-vector -1e35 0 0)
               :equality-matrix A
               :equality-vector ubA
               :solve-lp-mode solve-lp-mode
               )))
    (warn ";; solution ~A => ~A~%"
          ret
          (setq ret
                (eps-v= (float-vector 2 0 0) ret)))
    ;; correct solution
    ret
    ))

;;
(defun demo-eus-qpOASES-all
  (&key (mode :qp/lp))
  "Demo for all QP and LP."
  (let ((qp-mode (case mode
                       (:qp/lp :qp)
                       (:sqp/slp-with-hotstart :sqp-with-hotstart)))
        (lp-mode (case mode
                       (:qp/lp :lp)
                       (:sqp/slp-with-hotstart :slp-with-hotstart))))
    (and (demo-eus-qpOASES1 :solve-qp-mode qp-mode)
         (demo-eus-qpOASES2 :solve-qp-mode qp-mode)
         (demo-eus-qpOASES3 :solve-qp-mode qp-mode)
         (demo-eus-qpOASES4 :solve-qp-mode qp-mode)
         (demo-eus-qpOASES5 :solve-lp-mode lp-mode)
         (demo-eus-qpOASES6 :solve-lp-mode lp-mode)
         (demo-eus-qpOASES7 :solve-lp-mode lp-mode))
    ))

(warn ";; (demo-eus-qpOASES1) ;; ~A~%" (documentation 'demo-eus-qpOASES1))
(warn ";; (demo-eus-qpOASES2) ;; ~A~%" (documentation 'demo-eus-qpOASES2))
(warn ";; (demo-eus-qpOASES3) ;; ~A~%" (documentation 'demo-eus-qpOASES3))
(warn ";; (demo-eus-qpOASES4) ;; ~A~%" (documentation 'demo-eus-qpOASES4))
(warn ";; (demo-eus-qpOASES5) ;; ~A~%" (documentation 'demo-eus-qpOASES5))
(warn ";; (demo-eus-qpOASES6) ;; ~A~%" (documentation 'demo-eus-qpOASES6))
(warn ";; (demo-eus-qpOASES7) ;; ~A~%" (documentation 'demo-eus-qpOASES7))
(warn ";; (demo-eus-qpOASES-all) ;; ~A~%" (documentation 'demo-eus-qpOASES-all))
(warn ";; (demo-eus-qpOASES-all :mode :sqp/slp-with-hotstart) ;; ~A with hotstart~%" (documentation 'demo-eus-qpOASES-all))
