(defvar *prioritized-inverse-kinematics-files*
  (list "util"
        "inverse-kinematics-variable"
        "inverse-kinematics-task"
        "prioritized-inverse-kinematics"
        ))

(dolist (f *prioritized-inverse-kinematics-files*)
  (compiler::compile-file-if-src-newer (format nil "~A/prioritized-inverse-kinematics/euslisp/~A" (ros::resolve-ros-path "package://eus_qp") f))
  )
(load (format nil "~A/prioritized-inverse-kinematics/euslisp/prioritized-inverse-kinematics.so" (ros::resolve-ros-path "package://eus_qp")))
