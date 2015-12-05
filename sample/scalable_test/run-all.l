#!/usr/bin/env roseus

(require "gen-ik-net.l")

(dolist (param
	 (list (list 9 100 2 3 6 2 100)
	       (list 9 100 3 3 6 3 100)
	       (list 9 100 4 3 6 4 100)
	       (list 9 150 2 3 6 2 150)
	       (list 9 150 3 3 6 3 150)
	       (list 9 150 4 3 6 4 150)))
  (fileoutput-layer-string "ik_net.prototxt" (apply 'gen-redundancy-ik-net param))
  (unix::system "./gen_test_dir.sh &")
  (unix::sleep 3))


