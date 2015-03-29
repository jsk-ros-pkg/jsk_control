;; Parameter file for footstep planning

(setq *footstep-parameters*
      `((:foot-size . ((:x . 207)
                       (:z . 1)
                       (:y . 135)))
        (:default-projection-limit
          ((:z . 210)
           (:roll . 15)
           (:pitch . 15)))
        (:successors
          (:transform . ((:x . -80)  (:y . 210) (:theta . 0.0)))
          (:transform . ((:x . -40)  (:y . 248) (:theta . 4.58366)))
          (:transform . ((:x . -27)  (:y . 206) (:theta . 6)))
          (:transform . ((:x . 0.0)  (:y . 190) (:theta . 0.0)))
          (:transform . ((:x . 0.0)  (:y . 270) (:theta . 0.0)))
          (:transform . ((:x . 27)   (:y . 206) (:theta . -7)))
          (:transform . ((:x . 27)   (:y . 206) (:theta . 7)))
          (:transform . ((:x . 52.5) (:y . 260) (:theta . 0.0)))
          (:transform . ((:x . 75)   (:y . 210) (:theta . 0.0)))
          (:transform . ((:x . 100)  (:y . 210) (:theta . 0.0)))
          ,@(reduce #'append
                    (mapcar #'(lambda (x)
                                (list (cons :transform
                                            (list (cons :x x)
                                                  (cons :y 210)
                                                  (cons :theta 5)))
                                      (cons :transform
                                            (list (cons :x x)
                                                  (cons :y 210)
                                                  (cons :theta -5)))
                                      (cons :transform
                                            (list (cons :x x)
                                                  (cons :y 210)
                                                  (cons :theta 0)))))
                            '(250
                              ;;240
                              230
                              ;;220
                              210
                              200
                              ;; 190
                              ;; 180
                              ;; 170
                              ;; 160
                              150
                              ;; 140
                              ;; 130
                              ;; 120
                              ;; 110
                              100
                              ;; 90
                              ;; 80
                              ;; 70
                              ;; 60
                              50
                              ;; 40
                              ;; 30
                              ;; 20
                              10
                              0)))
          )))
