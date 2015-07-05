#!/usr/bin/env roseus
;;(load "~/ros/hydro_parent/src/jskeus/irteus/irtgraph.l")

(setq gr (instance graph :init))
(setq width 10)
(setq depth 1000)
(dotimes (d depth)
  (dotimes (w width)
    (let ((a (instance node :init (read-from-string (format nil "~A_~A" d w)))))
      (send gr :add-node a)
      (if (not (= w 0))
          (let ((from-name (read-from-string (format nil "~A_~A" d (1- w))))
                (to-name (read-from-string (format nil "~A_~A" d w))))
            (send gr :add-arc-from-to (send gr :node from-name) (send gr :node to-name))
            (send gr :add-arc-from-to (send gr :node to-name) (send gr :node from-name))
            ))
      (if (not (= d 0))
          (let ((from-name (read-from-string (format nil "~A_~A" (1- d) w)))
                (to-name (read-from-string (format nil "~A_~A" d w))))
            (send gr :add-arc-from-to (send gr :node from-name) (send gr :node to-name))
            ))
      )))
(format t "done build graph~%")
(send gr :start-state (send gr :node '0_0))
(send gr :goal-state (send gr :node (read-from-string (format nil "~A_~A" (1- depth) (1- width)))))

(setq asolver (instance breadth-first-graph-search-solver :init))
(format t "breadth first search~%")
(bench
 (send asolver :solve gr)))


(setq asolver (instance depth-first-graph-search-solver :init))
(format t "depth first search~%")
(bench
 (send asolver :solve gr)))

(setq asolver (instance best-first-graph-search-solver :init gr))
(format t "best first search~%")
(bench
 (send asolver :solve gr)))

