#!/usr/bin/env roseus


(defun generate-gif
    (&key
     (func-name-list)
     (look-all? t)
     )
  (dolist (func-name func-name-list)
    (null-output
     (setq *gif-name* (if (consp func-name) (elt func-name 1) func-name))
     (eval (read-from-string
            (format nil "
(~a
 :pre-process-func
 #'(lambda (sqp)
     (when (= (send sqp :iteration) 1)
       (setup-gif-animation :look-all? ~a))
     (save-image-for-gif-animation)
     )
 :visualize-callback-func
 #'save-image-for-gif-animation
 :visualize-loop? nil
 :output-filename nil
 :graph-filename nil
 )
" (if (consp func-name) (elt func-name 0) func-name)
  look-all?
  )))
     (create-gif-animation :save-dir (ros::resolve-ros-path "package://eus_qp/prioritized-inverse-kinematics/doc/images"))
    )
   (format t "###### ~a~%" *gif-name*)
   (format t "```~%(~a)~%```~%" (if (consp func-name) (elt func-name 0) func-name))
   (format t "![~a](doc/images/~a.gif)~%~%" *gif-name* *gif-name*)
   )
  )

(defun setup-gif-animation
    (&key
     (name *gif-name*)
     (look-all? t)
     (manual-adjust? nil)
     )
  (setq *gif-image-idx* 10000)
  (send *irtviewer* :resize 600 600)
  (when look-all?
    (send *irtviewer* :look-all))
  (send *irtviewer* :draw-objects)
  (when manual-adjust?
    (warning-message 1 "[gif] press Key if ready to start~%")
    (while t
      (x::window-main-one)
      (unix:usleep (* 100 1000))
      (when (kbhit)
        (return-from nil nil))
      )
    )
  (unix:system (format nil "rm -rf /tmp/eus_qp/prioritized-inverse-kinematics/~a" name))
  (unix:system (format nil "mkdir -p /tmp/eus_qp/prioritized-inverse-kinematics/~a/figs" name))
  (unix:usleep (* 100 1000))
  )

(defun save-image-for-gif-animation
    (&key
     (name *gif-name*)
     )
  (send *irtviewer* :viewer :viewsurface :write-to-image-file
        (format nil "/tmp/eus_qp/prioritized-inverse-kinematics/~a/figs/~a.png" name *gif-image-idx*))
  (incf *gif-image-idx*)
  )

(defun create-gif-animation
    (&key
     (name *gif-name*)
     (save-dir "/tmp")
     )
  (let* ((gif-path
          (format nil "~a/~a.gif" save-dir name))
         )
    (warning-message 1 "[gif] saving gif animation file to ~a~%" gif-path)
    (unix:system
     (format nil "convert -delay 6 -loop 0 -fuzz 5% -layers Optimize /tmp/eus_qp/prioritized-inverse-kinematics/~a/figs/*.png ~a" name gif-path))
    ))
