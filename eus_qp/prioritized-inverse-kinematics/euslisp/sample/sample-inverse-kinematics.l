(load "package://eus_qp/prioritized-inverse-kinematics/euslisp/prioritized-inverse-kinematics.l")

(defun setup ()
  (unless (boundp '*robot*)
    (load "irteus/demo/sample-robot-model.l")
    (setq *robot* (instance sample-robot :init)))
  (unless (boundp '*irtviewer*)
    (make-irtviewer))
  (objects (list *robot*))
  )

(defun test-inverse-kinematics
 (&key
    (debug-view :no-message)
    )
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))

  (prioritized-inverse-kinematics
   (list (list (instance move-target-task :init
                         (send *robot* :larm :end-coords)
                         (make-coords :pos #F(300 100 600)))))
   (mapcar #'(lambda (j) (instance joint-variable :init j))
           (send *robot* :larm :joint-list))
   :debug-view debug-view
   )
  )
