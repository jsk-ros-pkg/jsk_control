contact_states_observer
=====================

Euslisp server to check grasping and contact states.  

## Grasp state checking
For grasping state checking, use hand joint angle topics.  
First, users should configure setting using `:set-grasp-state-param` such as joint angles for success-candidates and failure-candidates and checked joint names.  
Then, observer program checks the difference between candidates and current hand joint angles and find the nearest candidate.  
Finally, observer program publishes results and users obtain them from `:get-contact-states`.
Setting parameter is as follows:
```
(list
  (list :rarm
      ;; Success and failure candidates
      (list
       (list :success #f(0.0 0.0 0.0 0.0 0.0 0.0))
       (list :fail #f(90.0 90.0 0.0 10.0 -20.0 -20.0))
       )
      ;; Checked joint names
      (list :t-1y :t-1p :f2-2r)
    ) ;; end of rarm setting
  (list :larm
    ...
    ) ;; end of larm setting
 ) ;; end of list
```


## Contact force checking
For contact force state checking, use force sensor value and thresholding.  
Observer program checks whether current force exceeds the threshold.  
Observer program publishes results and users obtain them from `:get-contact-states`.

## Use with real robot

* Start real robot (rtcd + ros bridge + servoOn)
* Run the following program on roseus

```
(progn
  (ros::roseus "test_grasp_check")
  ;; ROBOT Initialization. This is HRP2JSKNTS sample
  (load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknts-interface.l")
  (unless (boundp '*robot*)
    (hrp2jsknts-init)
    (setq *robot* *hrp2jsknts*))
  ;; Loading of Contact State Client
  (load "package://contact_states_observer/euslisp/contact-states-client.l")
  (setq csc (instance contact-state-client :init))
  ;; Set grasp parameter
  (send csc :set-grasp-state-param
        (list
         ;; Do not check larm. Always returns "Off"
         (list :larm)
         ;; Set rarm parameter.
         ;;   :success is hand's reset-pose, corresponding to "On" flag
         ;;   :fail is hand's hook-pose, corresponding to "Off" flag
         ;;   Check joint angles for t-1y, t-1p, f2-2r joints
         (list :rarm
               (list
                (list :success #f(0.0 0.0 0.0 0.0 0.0 0.0))
                (list :fail #f(90.0 90.0 0.0 10.0 -20.0 -20.0))
                )
               (list :t-1y :t-1p :f2-2r))))
  ;; Test1. Set reset-pose. This should return '("On" "Off")
  (send *robot* :hand :rarm :reset-pose)
  (send *ri* :hand-angle-vector (send *robot* :hand-angle-vector) 2000)
  (send *ri* :hand-wait-interpolation)
  (unix:usleep 200000) ;; Wait for hand motion stopping
  (format t ";; '(:rarm :larm) = ~A~%" (send csc :get-contact-states '(:rarm :larm)))
  ;; Test2. Set hook-pose. This should return '("Off" "Off")
  (send *robot* :hand :rarm :hook-pose)
  (send *ri* :hand-angle-vector (send *robot* :hand-angle-vector) 2000)
  (send *ri* :hand-wait-interpolation)
  (unix:usleep 200000) ;; Wait for hand motion stopping
  (format t ";; '(:rarm :larm) = ~A~%" (send csc :get-contact-states '(:rarm :larm)))
  )
```

## Publishing Topics
* `ore_contact_states` (`contact_states_observer/OREContactStates`)  
  Array of object, robot, environment contact state.  
  A contact state consists of  
   - name : for example, rleg, rarm,..  
   - state : On/Off (for grasping, foot, and hand contact)
   - mode : grasp, contact-force, ...
* `contact_grasp_states` (`hrpsys_ros_bridge/ContactStatesStamped`)  
  Contact states for all contact point (for example, rleg, lleg, rarm, larm)  
* `grasp_states` (`contact_states_observer/GraspState`)
  Grasp state
