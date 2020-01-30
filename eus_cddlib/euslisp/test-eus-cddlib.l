(load (format nil "~A/euslisp/eus-cddlib-compiled.l" (ros::resolve-ros-path "package://eus_cddlib")))


;; |0| + | 1  0| x >= 0
;; |0|   | 0  1|
;; |1|   |-1 -1|
;; |1|   | 0 -1|
;;
;; x =|0 1  0 -1|y + | | z
;;    |1 0 -1  0|    | |
;; sum y = 1, y >= 0, z >= 0
(defun test1 ()
  (let* ((A (matrix #F(1 0)
                   #F(0 1)
                   #F(-1 0)
                   #F(0 -1)))
         (b (float-vector 0
                         0
                         1
                         1))
         (ret1 (cddlib-H-to-V
                :A_ineq A
                :b_ineq b
                :verbose 1))
         (ret2 (cddlib-V-to-H
                :V (elt ret1 0)
                :R (elt ret1 1)
                :verbose 1))
         )
    (print ret1)
    (print ret2)
    )
  )



(defun test2 ()
  (let* ((A (matrix #F(0 0 1 0 0 0)
                    #F(1 0 0.2 0 0 0)
                    #F(-1 0 0.2 0 0 0)
                    #F(0 1 0.2 0 0 0)
                    #F(0 -1 0.2 0 0 0)
                    #F(0 0 0.09 1 0 0)
                    #F(0 0 0.09 -1 0 0)
                    #F(0 0 0.05 0 1 0)
                    #F(0 0 0.05 0 -1 0)
                    #F(0 0 0.01 0 0 1)
                    #F(0 0 0.01 0 0 -1)))
         (b (float-vector 0
                          0
                          0
                          0
                          0
                          0
                          0
                          0
                          0
                          0
                          0))
         (ret1 (cddlib-H-to-V
                :A_ineq A
                :b_ineq b
                :verbose 1))
         (ret2 (cddlib-V-to-H
                :V (elt ret1 0)
                :R (elt ret1 1)
                :verbose 1))
         )
    (format-array (elt ret1 0))
    (format-array (elt ret1 1))
    (format-array (elt ret2 0))
    (format-array (elt ret2 1))
    (format-array (elt ret2 2))
    (format-array (elt ret2 3))
    )
  )
