#!/usr/bin/env roseus

(load "fullbody-ik-client.l")

(setq ik-client-test (instance fullbody-ik-client :init))

(let ((test (unix::getenv "IK_CLIENT_TEST")))
  (cond
   ((null test))
   ((substringp "hrp2jsknt" test)
    (send ik-client-test :hrp2jsknt-test))
   ((substringp "hrp2jsk" test)
    (send ik-client-test :hrp2jsk-test))
   ((substringp "pr2" test)
    (send ik-client-test :pr2-test))
   ((substringp "atlas" test)
    (send ik-client-test :atlas-test))
   ))

