(compiler::compile-file-if-src-newer (format nil "~A/euslisp/eus-cddlib" (ros::resolve-ros-path "package://eus_cddlib")))
(load (format nil "~A/euslisp/eus-cddlib.so" (ros::resolve-ros-path "package://eus_cddlib")))
