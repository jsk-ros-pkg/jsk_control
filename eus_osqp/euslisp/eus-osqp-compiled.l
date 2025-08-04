(compiler::compile-file-if-src-newer (format nil "~A/euslisp/eus-osqp" (ros::resolve-ros-path "package://eus_osqp")))
(load (format nil "~A/euslisp/eus-osqp.so" (ros::resolve-ros-path "package://eus_osqp")))
